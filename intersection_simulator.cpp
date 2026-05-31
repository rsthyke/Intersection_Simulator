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
