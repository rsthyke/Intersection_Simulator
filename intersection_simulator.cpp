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
    void paint(HDC hdc, int W, int H) {
        std::lock_guard<std::mutex> lk(mtx);

        RECT full{ 0, 0, W, H };
        HBRUSH bg = CreateSolidBrush(RGB(232, 238, 230));
        FillRect(hdc, &full, bg);
        DeleteObject(bg);
        SetBkMode(hdc, TRANSPARENT);

        HFONT fHead = CreateFontA(20, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                                   OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                                   DEFAULT_PITCH | FF_SWISS, "Segoe UI");
        HFONT fSmall = CreateFontA(14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                                    OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                                    DEFAULT_PITCH | FF_SWISS, "Segoe UI");
        HFONT fTiny = CreateFontA(11, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                                   OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                                   DEFAULT_PITCH | FF_SWISS, "Segoe UI");
        HFONT oldFont = (HFONT)SelectObject(hdc, fSmall);

        const int cx0  = W / 2 - 70;
        const int cy0  = H / 2 + 20;
        const int box  = 86;
        const int half = 28;
        const int lane = 13;
        int vArm = (H - 170) / 2; if (vArm < 110) vArm = 110;
        int hArm = (W - 360) / 2; if (hArm < 160) hArm = 160;
        COLORREF roadCol = RGB(74, 78, 84);

        // roads
        { RECT v{ cx0 - half, cy0 - vArm, cx0 + half, cy0 + vArm };
          HBRUSH b = CreateSolidBrush(roadCol); FillRect(hdc, &v, b); DeleteObject(b); }
        { RECT h{ cx0 - hArm, cy0 - half, cx0 + hArm, cy0 + half };
          HBRUSH b = CreateSolidBrush(roadCol); FillRect(hdc, &h, b); DeleteObject(b); }

        // dashed centre lines (skip the box)
        {
            HPEN dash = CreatePen(PS_SOLID, 2, RGB(245, 210, 70));
            HPEN oldp = (HPEN)SelectObject(hdc, dash);
            for (int y = cy0 - vArm; y < cy0 + vArm; y += 26) {
                if (y > cy0 - box / 2 - 6 && y < cy0 + box / 2 + 6) continue;
                MoveToEx(hdc, cx0, y, nullptr); LineTo(hdc, cx0, y + 13);
            }
            for (int x = cx0 - hArm; x < cx0 + hArm; x += 26) {
                if (x > cx0 - box / 2 - 6 && x < cx0 + box / 2 + 6) continue;
                MoveToEx(hdc, x, cy0, nullptr); LineTo(hdc, x + 13, cy0);
            }
            SelectObject(hdc, oldp);
            DeleteObject(dash);
        }

        // the junction box (red when a car is crossing)
        {
            bool occ = !occupantName.empty();
            RECT bx{ cx0 - box / 2, cy0 - box / 2, cx0 + box / 2, cy0 + box / 2 };
            HBRUSH bb = CreateSolidBrush(occ ? RGB(201, 64, 64) : RGB(96, 100, 106));
            FillRect(hdc, &bx, bb);
            DeleteObject(bb);
            HPEN pen = CreatePen(PS_SOLID, 2, RGB(235, 235, 235));
            HPEN op  = (HPEN)SelectObject(hdc, pen);
            HBRUSH ob = (HBRUSH)SelectObject(hdc, GetStockObject(NULL_BRUSH));
            Rectangle(hdc, bx.left, bx.top, bx.right, bx.bottom);
            SelectObject(hdc, ob); SelectObject(hdc, op);
            DeleteObject(pen);
        }

        // one traffic light on each road (green if that road is green)
        auto lightAt = [&](int dir, int x, int y) {
            bool g = (greenDir == dir);
            HBRUSH b = CreateSolidBrush(g ? RGB(40, 200, 90) : RGB(200, 60, 60));
            HBRUSH ob = (HBRUSH)SelectObject(hdc, b);
            HPEN pen = CreatePen(PS_SOLID, 2, RGB(25, 25, 25));
            HPEN op = (HPEN)SelectObject(hdc, pen);
            Ellipse(hdc, x - 9, y - 9, x + 9, y + 9);
            SelectObject(hdc, ob); SelectObject(hdc, op);
            DeleteObject(b); DeleteObject(pen);
        };
        lightAt(0, cx0 + half + 16, cy0 - box / 2 - 14);   // N
        lightAt(1, cx0 - half - 16, cy0 + box / 2 + 14);   // S
        lightAt(2, cx0 + box / 2 + 14, cy0 - half - 16);   // E
        lightAt(3, cx0 - box / 2 - 14, cy0 + half + 16);   // W

        // road names with the number of waiting cars, like "W  (4)"
        SetTextColor(hdc, RGB(35, 35, 40));
        auto label = [&](HFONT f, const std::string& s, int x, int y) {
            SelectObject(hdc, f);
            SIZE sz; GetTextExtentPoint32A(hdc, s.c_str(), (int)s.size(), &sz);
            TextOutA(hdc, x - sz.cx / 2, y - sz.cy / 2, s.c_str(), (int)s.size());
        };
        const char* dn[4] = { "N", "S", "E", "W" };
        int lblX[4] = { cx0, cx0, cx0 + hArm + 30, cx0 - hArm - 30 };
        int lblY[4] = { cy0 - vArm - 20, cy0 + vArm + 20, cy0 - 2, cy0 - 2 };
        for (int d = 0; d < 4; d++) {
            std::ostringstream s; s << dn[d] << "  (" << approachWaiting[d] << ")";
            label(fHead, s.str(), lblX[d], lblY[d]);
        }
        label(fSmall, "Inter-A", cx0 - box / 2 - 42, cy0 - box / 2 - 12);

        // ---- vehicles ----
        auto pointOn = [&](int dir, double s, int& x, int& y, bool& vert) {
            if (dir == 0)      { vert = true;  x = cx0 + lane; y = (int)((cy0 - vArm) + s * (2.0 * vArm)); }
            else if (dir == 1) { vert = true;  x = cx0 - lane; y = (int)((cy0 + vArm) - s * (2.0 * vArm)); }
            else if (dir == 2) { vert = false; y = cy0 - lane; x = (int)((cx0 + hArm) - s * (2.0 * hArm)); }
            else               { vert = false; y = cy0 + lane; x = (int)((cx0 - hArm) + s * (2.0 * hArm)); }
        };
        auto sOf = [](const VLive& v) -> double {
            if (v.state == VState::DONE) return 0.96;
            double base, span;
            if (v.leg == 0) { base = 0.00; span = 0.40; } else { base = 0.52; span = 0.44; }
            if (v.state == VState::DRIVING) {
                double frac = std::chrono::duration<double, std::milli>(Clock::now() - v.phaseStart).count()
                              / (double)v.phaseDur;
                frac = std::max(0.0, std::min(1.0, frac));
                return base + frac * span;
            }
            if (v.state == VState::QUEUED) return 0.42;     // at the stop line
            return 0.50;                                    // CROSSING: in the box
        };

        std::vector<int> qSeen(4, 0), dSeen(4, 0);
        auto shortId = [](const std::string& name) -> std::string {
            std::string::size_type p = name.find('-');
            std::string id = (p == std::string::npos) ? "" : name.substr(p + 1);
            char c = name.empty() ? '?' : name[0];
            return std::string(1, c) + id;
        };

        for (size_t i = 0; i < veh.size(); i++) {
            const VLive& v = veh[i];
            COLORREF col = colorFor(v.type);
            int x, y; bool vert = false;
            bool highlight = (v.state == VState::CROSSING);

            if (v.state == VState::QUEUED) {
                int k = qSeen[v.dir]++, gap = 30;
                if (v.dir == 0)      { vert = true;  x = cx0 + lane; y = cy0 - box / 2 - 10 - k * gap; }
                else if (v.dir == 1) { vert = true;  x = cx0 - lane; y = cy0 + box / 2 + 10 + k * gap; }
                else if (v.dir == 2) { vert = false; y = cy0 - lane; x = cx0 + box / 2 + 10 + k * gap; }
                else                 { vert = false; y = cy0 + lane; x = cx0 - box / 2 - 10 - k * gap; }
            } else if (v.state == VState::DONE) {
                double s = 0.96 - dSeen[v.dir]++ * 0.04;
                pointOn(v.dir, s, x, y, vert);
                col = RGB((GetRValue(col) + 200) / 2, (GetGValue(col) + 200) / 2, (GetBValue(col) + 200) / 2);
            } else {
                pointOn(v.dir, sOf(v), x, y, vert);
            }

            drawCar(hdc, x, y, col, highlight, vert);
            SelectObject(hdc, fTiny);
            SetTextColor(hdc, RGB(20, 20, 20));
            std::string tag = shortId(v.name);
            SIZE ts; GetTextExtentPoint32A(hdc, tag.c_str(), (int)tag.size(), &ts);
            TextOutA(hdc, x - ts.cx / 2, y - ts.cy / 2, tag.c_str(), (int)tag.size());
            SetTextColor(hdc, RGB(35, 35, 40));
        }

        // ---- header ----
        SelectObject(hdc, fHead);
        SetTextColor(hdc, RGB(20, 20, 25));
        double elapsed = std::chrono::duration<double>(Clock::now() - t0).count();
        std::ostringstream hdrs;
        hdrs << "Mode: " << (modeStr.empty() ? "(idle)" : modeStr)
             << "      t = " << std::fixed << std::setprecision(2) << elapsed
             << " s      Fuel consumed: " << std::setprecision(1) << fuelNow << " L";
        TextOutA(hdc, 16, 12, hdrs.str().c_str(), (int)hdrs.str().size());
        SelectObject(hdc, fSmall);
        SetTextColor(hdc, RGB(70, 70, 78));
        const char* sub = "Adaptive signal gives GREEN to the road whose waiting traffic wastes the most fuel "
                          "(trucks first). Keys: [1] Adaptive   [2] Fixed";
        TextOutA(hdc, 16, 36, sub, (int)strlen(sub));

        // ---- comparison panel (top-right) ----
        {
            int px = W - 320, py = 60;
            SetTextColor(hdc, RGB(35, 35, 40));
            const char* tt = "TOTAL FUEL CONSUMED";
            TextOutA(hdc, px, py, tt, (int)strlen(tt));
            const char* nm[2] = { "Adaptive", "Fixed   " };
            for (int m = 0; m < 2; m++) {
                std::ostringstream s;
                s << nm[m] << ": ";
                if (haveMode[m]) s << fmt(modeFuel[m]) << " L   (truck wait " << modeTruck[m] << " ms)";
                else             s << "-- run it --";
                std::string line = s.str();
                TextOutA(hdc, px, py + 20 + m * 18, line.c_str(), (int)line.size());
            }
            if (haveMode[0] && haveMode[1]) {
                std::ostringstream s;
                double save = modeFuel[1] - modeFuel[0];
                s << "=> Adaptive saves " << fmt(save) << " L of fuel";
                std::string line = s.str();
                SetTextColor(hdc, RGB(20, 110, 40));
                TextOutA(hdc, px, py + 20 + 2 * 18 + 4, line.c_str(), (int)line.size());
            }
        }

        // ---- legend (bottom-left) ----
        SetTextColor(hdc, RGB(35, 35, 40));
        struct LI { std::string name; COLORREF col; };
        LI legend[3] = { { "Car", RGB(30,144,255) }, { "Truck", RGB(255,140,0) }, { "Ambulance", RGB(220,20,60) } };
        int lx = 16, ly = H - 30;
        for (auto& e : legend) {
            drawCar(hdc, lx + 13, ly + 7, e.col, false, false);
            TextOutA(hdc, lx + 32, ly, e.name.c_str(), (int)e.name.size());
            lx += 32 + (int)e.name.size() * 8 + 28;
        }
        {
            HBRUSH gb = CreateSolidBrush(RGB(40, 200, 90));
            Ellipse(hdc, lx, ly, lx + 16, ly + 16);
            HBRUSH ob = (HBRUSH)SelectObject(hdc, gb); SelectObject(hdc, ob); DeleteObject(gb);
            HBRUSH gb2 = CreateSolidBrush(RGB(40, 200, 90));
            RECT r1{ lx, ly, lx + 16, ly + 14 }; FillRect(hdc, &r1, gb2); DeleteObject(gb2);
            TextOutA(hdc, lx + 22, ly, "green", 5); lx += 22 + 5 * 8 + 20;
            HBRUSH rb = CreateSolidBrush(RGB(200, 60, 60));
            RECT r2{ lx, ly, lx + 16, ly + 14 }; FillRect(hdc, &r2, rb); DeleteObject(rb);
            TextOutA(hdc, lx + 22, ly, "red", 3);
        }

        SelectObject(hdc, oldFont);
        DeleteObject(fHead);
        DeleteObject(fSmall);
        DeleteObject(fTiny);
    }
#endif
};

LiveBoard         live;
std::atomic<bool> g_closed{false};     // window closed by the user
std::atomic<int>  g_request{0};        // GUI -> main: 1 = run adaptive, 2 = run fixed

// Vehicle: the base class for all cars.
class Vehicle {
protected:
    int       id;
    int       x, y;
    int       speed;
    double    fuelCost;      // fuel used per second while stopped (L)
    Direction from;
    long long wait_ms = 0;

public:
    Vehicle(int id_, int x_, int y_, int speed_, double fuelCost_, Direction from_)
        : id(id_), x(x_), y(y_), speed(speed_), fuelCost(fuelCost_), from(from_) {}
    virtual ~Vehicle() = default;

    virtual void move(int targetX, int targetY) {
        if      (x < targetX) x += speed;
        else if (x > targetX) x -= speed;
        if      (y < targetY) y += speed;
        else if (y > targetY) y -= speed;
    }

    // How much fuel this car wastes for each second it waits.
    // Trucks waste much more, so it is better to not stop them.
    virtual double calculateFuelLoss() const { return fuelCost; }

    virtual Priority    getPriority() const { return Priority::NORMAL; }
    virtual std::string getName()     const { return "Car-" + std::to_string(id); }
    virtual std::string typeName()    const { return "Car"; }
    virtual int         crossTimeMs() const { return 250; }

    int       getId()     const { return id; }
    Direction getFrom()   const { return from; }
    long long getWaitMs() const { return wait_ms; }
    void      addWaitMs(long long ms) { wait_ms += ms; }
    void      resetWaitMs()           { wait_ms = 0; }
};

class Truck : public Vehicle {
    double loadFactor;
    double momentumPenalty;
public:
    Truck(int id_, int x_, int y_, int speed_, double fuelCost_, Direction from_, double loadFactor_)
        : Vehicle(id_, x_, y_, speed_, fuelCost_, from_),
          loadFactor(loadFactor_), momentumPenalty(loadFactor_ * fuelCost_) {}
    double      calculateFuelLoss() const override { return momentumPenalty; }
    Priority    getPriority() const override { return Priority::HIGH; }
    std::string getName()     const override { return "Truck-" + std::to_string(id); }
    std::string typeName()    const override { return "Truck"; }
    int         crossTimeMs() const override { return 600; }
};

class Ambulance : public Vehicle {
public:
    Ambulance(int id_, int x_, int y_, int speed_, double fuelCost_, Direction from_)
        : Vehicle(id_, x_, y_, speed_, fuelCost_, from_) {}
    Priority    getPriority() const override { return Priority::EMERGENCY; }
    std::string getName()     const override { return "Ambulance-" + std::to_string(id); }
    std::string typeName()    const override { return "Ambulance"; }
    int         crossTimeMs() const override { return 150; }
};

// SignalJunction: the four-way junction and its traffic light.
//
// A mutex keeps it safe: only one road is green (greenDir) and only one
// car is in the box at a time. A car waits on the condition variable
// until its road is green and the box is free, then it crosses. The fuel
// a car uses = its fuel weight times the seconds it waited at the red
// light. So less waiting (above all for trucks) means less fuel.
class SignalJunction {
    std::mutex              mtx;
    std::condition_variable cv;
    int    greenDir = -1;
    bool   occupied = false;
    int    waiting[4]      = {0,0,0,0};
    int    waitEmg[4]      = {0,0,0,0};   // how many ambulances wait on each road
    double waitFuel[4]     = {0,0,0,0};   // total fuel weight of the cars waiting on each road
    double fuel_consumed   = 0.0;

public:
    void reset() {
        std::lock_guard<std::mutex> lk(mtx);
        greenDir = -1; occupied = false; fuel_consumed = 0.0;
        for (int i = 0; i < 4; i++) { waiting[i] = 0; waitEmg[i] = 0; waitFuel[i] = 0.0; }
    }

    // setGreen(): controller switches the green light to `dir`.
    void setGreen(int dir) {
        std::lock_guard<std::mutex> lk(mtx);
        if (greenDir != dir) { greenDir = dir; live.setGreen(dir); cv.notify_all(); }
    }

    // Find the road whose waiting cars would waste the most fuel.
    // Returns -1 if no car waits.
    int bestDemandDir() {
        std::lock_guard<std::mutex> lk(mtx);
        int best = -1; double bestVal = 0.0;
        for (int d = 0; d < 4; d++) {
            if (waiting[d] > 0 && waitFuel[d] > bestVal) { bestVal = waitFuel[d]; best = d; }
        }
        return best;
    }
    // Find a road with a waiting ambulance, or -1. An ambulance gets
    // green at once, before the fuel rule.
    int emergencyDir() {
        std::lock_guard<std::mutex> lk(mtx);
        for (int d = 0; d < 4; d++) if (waitEmg[d] > 0) return d;
        return -1;
    }
    int  waitingOf(int dir) { std::lock_guard<std::mutex> lk(mtx); return waiting[dir]; }
    bool busy()             { std::lock_guard<std::mutex> lk(mtx); return occupied; }
    double getFuelConsumed(){ std::lock_guard<std::mutex> lk(mtx); return fuel_consumed; }

    // A car calls this. It waits until its road is green and the box is
    // free, adds the fuel it used while waiting, then crosses. It returns
    // how long it waited.
    long long cross(Vehicle& v, int dir, int idx) {
        std::unique_lock<std::mutex> lk(mtx);
        bool emg = (v.getPriority() == Priority::EMERGENCY);
        waiting[dir]++;  waitFuel[dir] += v.calculateFuelLoss();  if (emg) waitEmg[dir]++;
        live.setApproach(dir, waiting[dir]);

        auto t0 = Clock::now();
        cv.wait(lk, [&] { return greenDir == dir && !occupied; });
        long long waited = std::chrono::duration_cast<Ms>(Clock::now() - t0).count();

        waiting[dir]--;  waitFuel[dir] -= v.calculateFuelLoss();  if (emg) waitEmg[dir]--;
        live.setApproach(dir, waiting[dir]);
        occupied = true;
        live.setOccupant(v.getName());

        // fuel this car used = its weight x the seconds it waited
        double consumed = v.calculateFuelLoss() * (waited / 1000.0);
        fuel_consumed += consumed;
        live.addFuel(consumed);
        logLine("[CROSS] " + dirName(dir) + " GREEN -> " + v.getName() +
                "  waited " + std::to_string(waited) + " ms  (+" + fmt(consumed) + " L)");

        lk.unlock();
        live.setCrossing(idx, 0);
        std::this_thread::sleep_for(Ms(v.crossTimeMs()));
        lk.lock();
        occupied = false;
        live.setOccupant("");
        cv.notify_all();
        return waited;
    }
};

// controllerLoop(): the traffic light's brain (runs in its own thread).
//   adaptive == true : give green to the road that wastes the most fuel,
//                      keep it until that road is empty, then check again.
//   adaptive == false: give green in a fixed order (N -> S -> E -> W),
//                      a fixed time each, and do not look at the traffic.
void controllerLoop(SignalJunction* j, bool adaptive, std::atomic<bool>* stop) {
    using namespace std::chrono_literals;
    int rr = 0;
    while (!stop->load()) {
        if (adaptive) {
            // An ambulance gets green first. If none, give green to the
            // road that wastes the most fuel.
            int em  = j->emergencyDir();
            int dir = (em >= 0) ? em : j->bestDemandDir();
            if (dir < 0) { std::this_thread::sleep_for(40ms); continue; }
            j->setGreen(dir);
            auto start = Clock::now();
            while (!stop->load()) {
                std::this_thread::sleep_for(50ms);
                if (j->waitingOf(dir) == 0 && !j->busy()) break;             // road is empty
                int e2 = j->emergencyDir();
                if (e2 >= 0 && e2 != dir) break;                             // ambulance on another road
                if (std::chrono::duration<double>(Clock::now() - start).count() > 6.0) break;
            }
        } else {
            int dir = rr++ % 4;
            j->setGreen(dir);
            std::this_thread::sleep_for(1100ms);   // fixed time, ignores the traffic
        }
    }
}

// Vehicle thread: drive to the junction, wait for green, cross, drive out.
void vehicleThread(Vehicle* v, SignalJunction* j, int idx) {
    int dir = (int)v->getFrom();
    // When each car arrives. The trucks come early and together. The cars
    // come later and spread out. So a fixed light keeps green on empty
    // roads while the trucks wait.
    long long travel_ms;
    switch (v->getPriority()) {
        case Priority::HIGH:      travel_ms = 70 + (v->getId() * 13) % 90;    break; // trucks come first
        case Priority::EMERGENCY: travel_ms = 500;                            break; // ambulance
        default:                  travel_ms = 500 + (v->getId() * 37) % 2200; break; // cars come later
    }
    live.setDriving(idx, 0, travel_ms);
    std::this_thread::sleep_for(Ms(travel_ms));

    live.setQueued(idx, 0);
    long long waited = j->cross(*v, dir, idx);
    v->addWaitMs(waited);

    int exit_ms = 140 + (v->getId() * 31) % 220;
    live.setDriving(idx, 1, exit_ms);
    std::this_thread::sleep_for(Ms(exit_ms));
    live.setDone(idx, v->getWaitMs());
    logLine("[DONE]  " + v->getName() + " finished  total_wait=" +
            std::to_string(v->getWaitMs()) + " ms");
}
