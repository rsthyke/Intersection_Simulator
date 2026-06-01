// Intelligent Intersection Simulator
//
// A four-way junction with a smart traffic light. The light gives green
// to the road that wastes the most fuel (the road with the trucks). An
// ambulance always gets green. There is also a simple fixed light that
// only changes in order. We compare them: the smart light uses less fuel.
//
// One thread runs each car and one thread runs the light. A mutex keeps
// the junction safe (one car at a time). A condition variable lets a car
// sleep until its road is green. All cars are kept in one array.
//
// The window is interactive: press [1] for the smart light and [2] for
// the fixed light. It shows the total fuel for each one.

#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <algorithm>
#include <cstring>

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#endif

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

// Print to the screen and to the log file (results.txt).
std::mutex    print_mutex;
std::ofstream log_file;

void printLine(const std::string& msg) {
    std::lock_guard<std::mutex> lk(print_mutex);
    std::cout << msg << "\n";
    if (log_file.is_open()) log_file << msg << "\n";
}

// Write only to the log file, not to the screen.
void logLine(const std::string& msg) {
    std::lock_guard<std::mutex> lk(print_mutex);
    if (log_file.is_open()) log_file << msg << "\n";
}

// Make a text with one digit after the dot, like "19.1".
std::string fmt(double d) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(1) << d;
    return s.str();
}

// ---- Enums ----
enum class Direction { NORTH, SOUTH, EAST, WEST };   // 0,1,2,3 = N,S,E,W
enum class Priority  { NORMAL, HIGH, EMERGENCY };

std::string dirName(int d) {
    switch (d) {
        case 0: return "NORTH";
        case 1: return "SOUTH";
        case 2: return "EAST ";
        case 3: return "WEST ";
    }
    return "?";
}

// LiveBoard: holds the data for the live window.
//
// The car threads and the light write small updates here (which road is
// green, who is crossing, how many cars wait, total fuel). A GUI thread
// reads this and draws the junction about 30 times a second. It also
// keeps each mode's totals so we can compare them on the screen.
enum class VState { DRIVING, QUEUED, CROSSING, DONE };

struct VLive {
    std::string name, type;
    int    leg   = 0;                // 0 = going to junction, 1 = leaving
    int    dir   = 0;                // road: 0=N, 1=S, 2=E, 3=W
    VState state = VState::DRIVING;
    Clock::time_point phaseStart = Clock::now();
    long long phaseDur = 100000;     // ms, used to move the car while driving
    long long waitMs   = 0;
};

class LiveBoard {
    std::mutex mtx;
    std::vector<VLive> veh;
    int    greenDir = -1;            // road that is green now
    int    approachWaiting[4] = {0,0,0,0};
    std::string occupantName;
    std::string modeStr;
    double fuelNow = 0.0;            // fuel consumed so far in the current pass
    Clock::time_point t0;

    // remembered per-mode results (0 = adaptive, 1 = fixed)
    bool      haveMode[2]  = {false,false};
    double    modeFuel[2]  = {0,0};
    long long modeAmb[2]   = {0,0};
    long long modeTruck[2] = {0,0};
    long long modeCar[2]   = {0,0};

#ifdef _WIN32
    static COLORREF colorFor(const std::string& type) {
        if (type == "Truck")     return RGB(255, 140, 0);
        if (type == "Ambulance") return RGB(220, 20, 60);
        return RGB(30, 144, 255); // Car
    }
    static void drawCar(HDC hdc, int cx, int cy, COLORREF col, bool highlight, bool vertical) {
        int hw = vertical ? 8 : 13;
        int hh = vertical ? 13 : 8;
        HBRUSH b    = CreateSolidBrush(col);
        HBRUSH oldb = (HBRUSH)SelectObject(hdc, b);
        HPEN   pen  = CreatePen(PS_SOLID, highlight ? 3 : 1,
                                highlight ? RGB(255, 255, 255) : RGB(40, 40, 40));
        HPEN   oldp = (HPEN)SelectObject(hdc, pen);
        RoundRect(hdc, cx - hw, cy - hh, cx + hw, cy + hh, 7, 7);
        SelectObject(hdc, oldb);
        SelectObject(hdc, oldp);
        DeleteObject(b);
        DeleteObject(pen);
    }
#endif

public:
    void begin(const std::vector<std::string>& names, const std::vector<std::string>& types,
               const std::vector<int>& dirs, const std::string& mode) {
        std::lock_guard<std::mutex> lk(mtx);
        veh.assign(names.size(), VLive{});
        for (size_t i = 0; i < names.size(); i++) {
            veh[i].name       = names[i];
            veh[i].type       = types[i];
            veh[i].dir        = (i < dirs.size()) ? dirs[i] : 0;
            veh[i].phaseStart = Clock::now();
        }
        greenDir = -1;
        for (int i = 0; i < 4; i++) approachWaiting[i] = 0;
        occupantName.clear();
        modeStr  = mode;
        fuelNow  = 0.0;
        t0       = Clock::now();
    }

    void setDriving(int i, int leg, long long durMs) {
        std::lock_guard<std::mutex> lk(mtx);
        veh[i].state = VState::DRIVING; veh[i].leg = leg;
        veh[i].phaseStart = Clock::now();
        veh[i].phaseDur   = durMs > 0 ? durMs : 1;
    }
    void setQueued(int i, int leg)   { std::lock_guard<std::mutex> lk(mtx); veh[i].state = VState::QUEUED;   veh[i].leg = leg; }
    void setCrossing(int i, int leg) { std::lock_guard<std::mutex> lk(mtx); veh[i].state = VState::CROSSING; veh[i].leg = leg; }
    void setDone(int i, long long w) { std::lock_guard<std::mutex> lk(mtx); veh[i].state = VState::DONE;     veh[i].waitMs = w; }

    void setGreen(int dir)               { std::lock_guard<std::mutex> lk(mtx); greenDir = dir; }
    void setApproach(int dir, int count) { std::lock_guard<std::mutex> lk(mtx); if (dir >= 0 && dir < 4) approachWaiting[dir] = count; }
    void setOccupant(const std::string& n){ std::lock_guard<std::mutex> lk(mtx); occupantName = n; }
    void addFuel(double f)               { std::lock_guard<std::mutex> lk(mtx); fuelNow += f; }

    void recordMode(int m, double fuel, long long amb, long long truck, long long car) {
        std::lock_guard<std::mutex> lk(mtx);
        if (m < 0 || m > 1) return;
        haveMode[m] = true; modeFuel[m] = fuel;
        modeAmb[m] = amb; modeTruck[m] = truck; modeCar[m] = car;
    }

#ifdef _WIN32
