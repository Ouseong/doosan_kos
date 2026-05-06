// Direct DRFL servoj_rt streamer — bypasses dsr_hw_interface2.
//
// usage:
//   rt_streamer --test                  (J6 ±1° wiggle from current pose, 2s)
//   rt_streamer --swing <csv>           (stream CSV at 100 Hz)
//
// Pre-req: dsr_hw_interface2 / dsr_bringup2 NOT running (DRCF allows one client).
//
// build:  see scripts/build_rt_streamer.sh

#include <cstdio>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <array>
#include "DRFLEx.h"

using namespace DRAFramework;

static const char* ROBOT_IP = "192.168.137.100";

static void sleep_until(std::chrono::steady_clock::time_point t) {
    auto now = std::chrono::steady_clock::now();
    if (t > now) std::this_thread::sleep_until(t);
}

static std::vector<std::array<float, 6>> load_csv(const std::string& path) {
    std::vector<std::array<float, 6>> rows;
    std::ifstream f(path);
    if (!f.is_open()) { fprintf(stderr, "cannot open %s\n", path.c_str()); return rows; }
    std::string line;
    std::getline(f, line);   // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::array<float, 6> row;
        std::string cell;
        for (int i = 0; i < 6; ++i) { std::getline(ss, cell, ','); row[i] = std::stof(cell); }
        rows.push_back(row);
    }
    return rows;
}

int main(int argc, char** argv) {
    bool test_mode = false;
    std::string csv_path;
    if (argc >= 2 && std::string(argv[1]) == "--test") {
        test_mode = true;
    } else if (argc >= 3 && std::string(argv[1]) == "--swing") {
        csv_path = argv[2];
    } else {
        fprintf(stderr, "usage: %s --test  |  %s --swing <csv>\n", argv[0], argv[0]);
        return 1;
    }

    CDRFLEx drfl;
    printf("[1] open_connection(%s:12345)\n", ROBOT_IP);
    if (!drfl.open_connection(ROBOT_IP, 12345)) { fprintf(stderr, "open_connection failed\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("[2] manage_access_control(FORCE_REQUEST)\n");
    drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Ensure servo is ON and STATE = STANDBY
    printf("[3] set_robot_control(CONTROL_SERVO_ON) + wait STANDBY\n");
    bool standby = false;
    for (int retry = 0; retry < 10; ++retry) {
        ROBOT_STATE state = drfl.get_robot_state();
        printf("    state=%d (target STANDBY=%d)\n", (int)state, (int)STATE_STANDBY);
        if (state == STATE_STANDBY) { standby = true; break; }
        drfl.set_robot_control(CONTROL_SERVO_ON);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
    if (!standby) { fprintf(stderr, "robot not STANDBY, abort\n"); drfl.close_connection(); return 1; }

    printf("[4] set_safety_mode(AUTONOMOUS, EVENT_MOVE)\n");
    drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    printf("[5] connect_rt_control(%s:12347)\n", ROBOT_IP);
    if (!drfl.connect_rt_control(ROBOT_IP, 12347)) { fprintf(stderr, "connect_rt_control failed\n"); drfl.close_connection(); return 1; }

    printf("[6] set_rt_control_output (v1.0, 1ms, loss=4)\n");
    drfl.set_rt_control_output("v1.0", 0.001, 4);

    printf("[7] start_rt_control\n");
    if (!drfl.start_rt_control()) { fprintf(stderr, "start_rt_control failed\n"); drfl.disconnect_rt_control(); drfl.close_connection(); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Read current pose
    LPROBOT_POSE pPose = drfl.get_current_posj();
    if (!pPose) { fprintf(stderr, "get_current_posj null\n"); drfl.stop_rt_control(); drfl.disconnect_rt_control(); drfl.close_connection(); return 1; }
    float start_deg[6];
    for (int i = 0; i < 6; ++i) start_deg[i] = pPose->_fPosition[i];
    printf("    current pose (deg): %.1f %.1f %.1f %.1f %.1f %.1f\n",
           start_deg[0], start_deg[1], start_deg[2], start_deg[3], start_deg[4], start_deg[5]);

    // Higher envelopes for swing; --test still triggers visible J6 motion
    float vel_lim[6] = {120, 120, 180, 200, 200, 200};
    float acc_lim[6] = {300, 300, 400, 500, 500, 500};
    drfl.set_velj_rt(vel_lim);
    drfl.set_accj_rt(acc_lim);

    // Build trajectory
    std::vector<std::array<float, 6>> traj;
    int rate_hz = 100;
    if (test_mode) {
        // 2 s, J6 sinusoidal ±10° from current pose (visible to the eye)
        for (int i = 0; i < 200; ++i) {
            float t = i * 0.01f;
            float dj6 = 10.0f * std::sin(2.0f * M_PI * t / 2.0f);
            std::array<float, 6> row;
            for (int j = 0; j < 6; ++j) row[j] = start_deg[j];
            row[5] += dj6;
            traj.push_back(row);
        }
        // hold final pose 0.5 s
        for (int i = 0; i < 50; ++i) {
            std::array<float, 6> row;
            for (int j = 0; j < 6; ++j) row[j] = start_deg[j];
            traj.push_back(row);
        }
    } else {
        std::vector<std::array<float, 6>> swing = load_csv(csv_path);
        if (swing.empty()) { fprintf(stderr, "empty CSV\n"); return 1; }
        printf("    swing CSV %zu rows\n", swing.size());
        // Phase A: smooth approach from current pose to swing[0] over 3 s (300 samples)
        const int n_warmup = 300;
        for (int i = 0; i < n_warmup; ++i) {
            float a = (float)(i + 1) / n_warmup;   // 0→1 linear
            // smoothstep for gentler ends
            float s = a * a * (3.0f - 2.0f * a);
            std::array<float, 6> row;
            for (int j = 0; j < 6; ++j) row[j] = start_deg[j] * (1.0f - s) + swing[0][j] * s;
            traj.push_back(row);
        }
        // Phase B: swing CSV (1 s, 101 samples)
        for (auto& r : swing) traj.push_back(r);
        // Phase C: hold impact pose 0.5 s
        for (int i = 0; i < 50; ++i) traj.push_back(swing.back());
    }
    printf("[8] streaming %zu samples at %d Hz\n", traj.size(), rate_hz);

    // Stream
    auto period = std::chrono::microseconds(1000000 / rate_hz);
    auto next = std::chrono::steady_clock::now();
    float zero[6] = {0, 0, 0, 0, 0, 0};
    float servo_time = 0.02f;   // tight tracking: 2× streaming period (10ms) = 20ms smoothing window

    int n_ok = 0, n_fail = 0;
    for (size_t i = 0; i < traj.size(); ++i) {
        float pos[6];
        for (int j = 0; j < 6; ++j) pos[j] = traj[i][j];
        bool ok = drfl.servoj_rt(pos, zero, zero, servo_time);
        if (ok) n_ok++; else n_fail++;
        if (i < 5 || (i % 50 == 0)) {
            printf("    i=%3zu ok=%d pos=[%.1f %.1f %.1f %.1f %.1f %.1f]\n",
                   i, ok ? 1 : 0, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
        }
        next += period;
        sleep_until(next);
    }
    auto t_end = std::chrono::steady_clock::now();
    printf("[9] stream done. servoj_rt: %d ok, %d fail. settling 1s\n", n_ok, n_fail);

    // Read final pose to compare
    LPROBOT_POSE pPose2 = drfl.get_current_posj();
    if (pPose2) {
        printf("    final pose (deg): %.1f %.1f %.1f %.1f %.1f %.1f\n",
               pPose2->_fPosition[0], pPose2->_fPosition[1], pPose2->_fPosition[2],
               pPose2->_fPosition[3], pPose2->_fPosition[4], pPose2->_fPosition[5]);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Cleanup
    printf("[10] stop_rt_control + disconnect + close\n");
    drfl.stop_rt_control();
    drfl.disconnect_rt_control();
    drfl.close_connection();
    printf("done.\n");
    return 0;
}
