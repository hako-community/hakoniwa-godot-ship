// cpa_ozt.gd 検証用の参照値を hakoniwa-mujoco-ship の C++ 実装（cpa_ozt.hpp）から直接生成する。
//
//   pwsh -File tests/unit/ozt_parity/gen_reference.ps1   →  reference.json（コミット対象）
//
// ケースは固定シードの LCG で生成するので、何度実行しても同じ reference.json になる。
#include <cstdint>
#include <cstdio>
#include <string>

#include "ship/autonomy/cpa_ozt.hpp"

using hako::ship::autonomy::CpaOztCalculator;
using hako::ship::autonomy::Vector2D;

namespace {

uint64_t g_state = 20261022ULL;

double uniform(double lo, double hi) {
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    const double u = static_cast<double>(g_state >> 11) / 9007199254740992.0;  // [0, 1)
    return lo + (hi - lo) * u;
}

std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return buf;
}

std::string vec(const Vector2D& v) {
    return "[" + num(v.x) + ", " + num(v.y) + "]";
}

const char* boolean(bool b) { return b ? "true" : "false"; }

}  // namespace

int main() {
    std::printf("{\n  \"cpa\": [\n");
    const int kCpaCases = 60;
    for (int i = 0; i < kCpaCases; ++i) {
        const Vector2D own_pos{uniform(-3000, 3000), uniform(-3000, 3000)};
        Vector2D own_vel{uniform(-8, 8), uniform(-8, 8)};
        Vector2D tgt_pos{uniform(-3000, 3000), uniform(-3000, 3000)};
        Vector2D tgt_vel{uniform(-8, 8), uniform(-8, 8)};
        const double heading = uniform(-3.2, 6.4);
        double cpa_alarm = 926.0, tcpa_alarm = 900.0;
        if (i == 0) tgt_vel = own_vel;                       // 相対速度ゼロ分岐
        if (i == 1) tgt_vel = {0.0, 0.0};                     // 停止船
        if (i == 2) { own_vel = {0, 5}; tgt_vel = {0, -5}; }  // 舳先線と平行（det≈0 分岐）
        if (i == 3) { cpa_alarm = 3000.0; tcpa_alarm = 3600.0; }
        const auto m = CpaOztCalculator::compute_cpa(own_pos, own_vel, heading, tgt_pos, tgt_vel, cpa_alarm, tcpa_alarm);
        std::printf(
            "    {\"own_pos\": %s, \"own_vel\": %s, \"heading_rad\": %s, \"tgt_pos\": %s, \"tgt_vel\": %s,"
            " \"cpa_alarm_dist_m\": %s, \"tcpa_alarm_time_s\": %s,\n"
            "     \"cpa_distance_m\": %s, \"tcpa_seconds\": %s, \"bcr_distance_m\": %s, \"bct_seconds\": %s,"
            " \"bow_cross_ahead\": %s, \"is_closing\": %s, \"collision_risk\": %s}%s\n",
            vec(own_pos).c_str(), vec(own_vel).c_str(), num(heading).c_str(), vec(tgt_pos).c_str(), vec(tgt_vel).c_str(),
            num(cpa_alarm).c_str(), num(tcpa_alarm).c_str(),
            num(m.cpa_distance_m).c_str(), num(m.tcpa_seconds).c_str(), num(m.bcr_distance_m).c_str(), num(m.bct_seconds).c_str(),
            boolean(m.bow_cross_ahead), boolean(m.is_closing), boolean(m.collision_risk), i + 1 < kCpaCases ? "," : "");
    }
    std::printf("  ],\n  \"ozt\": [\n");
    const int kOztCases = 40;
    for (int i = 0; i < kOztCases; ++i) {
        // 危険領域が出やすいよう、他船は自船から 0.3〜2.5 km に置き、自船方向へ寄せた速度にする
        const Vector2D own_pos{uniform(-2000, 2000), uniform(-2000, 2000)};
        double own_speed = uniform(2.0, 8.0);
        const double r = uniform(300, 2500), brg = uniform(0, 6.283185307179586);
        const Vector2D tgt_pos{own_pos.x + r * std::sin(brg), own_pos.y + r * std::cos(brg)};
        const double tgt_course = brg + 3.141592653589793 + uniform(-1.2, 1.2);
        const double tgt_speed = uniform(0.0, 7.0);
        const Vector2D tgt_vel{tgt_speed * std::sin(tgt_course), tgt_speed * std::cos(tgt_course)};
        double safe = uniform(300, 926), horizon = uniform(300, 900);
        if (i == 0) own_speed = 0.05;  // 自船停止（危険領域なし分岐）
        const auto s = CpaOztCalculator::compute_ozt(own_pos, own_speed, tgt_pos, tgt_vel, safe, horizon);
        std::string poly = "[";
        for (size_t k = 0; k < s.danger_polygon.size(); ++k) {
            poly += (k ? ", " : "") + vec(s.danger_polygon[k]);
        }
        poly += "]";
        std::printf(
            "    {\"own_pos\": %s, \"own_speed_mps\": %s, \"tgt_pos\": %s, \"tgt_vel\": %s,"
            " \"safe_domain_m\": %s, \"time_horizon_s\": %s,\n"
            "     \"has_danger\": %s, \"min_danger_course_deg\": %s, \"max_danger_course_deg\": %s,\n"
            "     \"danger_polygon\": %s}%s\n",
            vec(own_pos).c_str(), num(own_speed).c_str(), vec(tgt_pos).c_str(), vec(tgt_vel).c_str(),
            num(safe).c_str(), num(horizon).c_str(),
            boolean(s.has_danger), num(s.min_danger_course_deg).c_str(), num(s.max_danger_course_deg).c_str(),
            poly.c_str(), i + 1 < kOztCases ? "," : "");
    }
    std::printf("  ]\n}\n");
    return 0;
}
