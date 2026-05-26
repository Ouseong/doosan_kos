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

// 19-column CSV: t, J1..J6 (deg), J1_dot..J6_dot (deg/s), J1_ddot..J6_ddot (deg/s²)
// 6-column legacy (pos only) is also accepted; vel/acc will be zero.
struct TrajFrame {
    float pos[6];
    float vel[6];
    float acc[6];
};

static std::vector<TrajFrame> load_csv(const std::string& path) {
    std::vector<TrajFrame> rows;
    std::ifstream f(path);
    if (!f.is_open()) { fprintf(stderr, "cannot open %s\n", path.c_str()); return rows; }
    std::string line;
    std::getline(f, line);   // header — detect column count
    bool has_vel_acc = (line.find("_dot") != std::string::npos);
    bool has_time    = (line[0] == 't');
    int  pos_offset  = has_time ? 1 : 0;

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        TrajFrame fr;
        memset(&fr, 0, sizeof(fr));
        int total = pos_offset + (has_vel_acc ? 18 : 6);
        std::vector<float> vals;
        for (int i = 0; i < total; ++i) {
            std::getline(ss, cell, ',');
            vals.push_back(std::stof(cell));
        }
        for (int j = 0; j < 6; ++j) fr.pos[j] = vals[pos_offset + j];
        if (has_vel_acc) {
            for (int j = 0; j < 6; ++j) fr.vel[j] = vals[pos_offset + 6 + j];
            for (int j = 0; j < 6; ++j) fr.acc[j] = vals[pos_offset + 12 + j];
        }
        rows.push_back(fr);
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
    std::vector<TrajFrame> traj;
    int rate_hz = 100;
    if (test_mode) {
        // 2 s, J6 sinusoidal ±10° — analytical vel/acc feedforward
        float omega = 2.0f * M_PI / 2.0f;   // 1 Hz sine
        float amp   = 10.0f;                 // ±10°
        for (int i = 0; i < 200; ++i) {
            float t = i * 0.01f;
            TrajFrame fr; memset(&fr, 0, sizeof(fr));
            for (int j = 0; j < 6; ++j) fr.pos[j] = start_deg[j];
            fr.pos[5] += amp * std::sin(omega * t);
            fr.vel[5]  = amp * omega * std::cos(omega * t);          // deg/s
            fr.acc[5]  = -amp * omega * omega * std::sin(omega * t); // deg/s²
            traj.push_back(fr);
        }
        // hold final pose 0.5 s
        for (int i = 0; i < 50; ++i) {
            TrajFrame fr; memset(&fr, 0, sizeof(fr));
            for (int j = 0; j < 6; ++j) fr.pos[j] = start_deg[j];
            traj.push_back(fr);
        }
    } else {
        std::vector<TrajFrame> swing = load_csv(csv_path);
        if (swing.empty()) { fprintf(stderr, "empty CSV\n"); return 1; }
        printf("    swing CSV %zu rows (vel/acc feedforward: %s)\n",
               swing.size(),
               (swing[0].vel[0] != 0 || swing[0].vel[1] != 0) ? "yes" : "pos-only");
        // Phase A: smoothstep from current pose to swing[0] over 3 s — vel/acc zero (slow phase)
        const int n_warmup = 300;
        for (int i = 0; i < n_warmup; ++i) {
            float a = (float)(i + 1) / n_warmup;
            float s = a * a * (3.0f - 2.0f * a);
            TrajFrame fr; memset(&fr, 0, sizeof(fr));
            for (int j = 0; j < 6; ++j) fr.pos[j] = start_deg[j] * (1.0f - s) + swing[0].pos[j] * s;
            traj.push_back(fr);
        }
        // Phase B: swing CSV with full feedforward
        for (auto& r : swing) traj.push_back(r);
        // Phase C: hold impact pose 0.5 s
        for (int i = 0; i < 50; ++i) {
            TrajFrame fr = swing.back();
            memset(fr.vel, 0, sizeof(fr.vel));
            memset(fr.acc, 0, sizeof(fr.acc));
            traj.push_back(fr);
        }
    }
    printf("[8] streaming %zu samples at %d Hz\n", traj.size(), rate_hz);

    // Stream
    auto period = std::chrono::microseconds(1000000 / rate_hz);
    auto next = std::chrono::steady_clock::now();
    float servo_time = 0.02f;   // tight tracking: 2× streaming period (10ms) = 20ms smoothing window

    int n_ok = 0, n_fail = 0;
    for (size_t i = 0; i < traj.size(); ++i) {
        bool ok = drfl.servoj_rt(traj[i].pos, traj[i].vel, traj[i].acc, servo_time);
        if (ok) n_ok++; else n_fail++;
        if (i < 5 || (i % 50 == 0)) {
            printf("    i=%3zu ok=%d pos=[%.1f %.1f %.1f %.1f %.1f %.1f]\n",
                   i, ok ? 1 : 0,
                   traj[i].pos[0], traj[i].pos[1], traj[i].pos[2],
                   traj[i].pos[3], traj[i].pos[4], traj[i].pos[5]);
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
