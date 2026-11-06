// veto_core_test.cpp — R1-R3 断言的 C++ 移植（2026-11-06）
//
// 对照 Python 版 test_veto_regression.py 逐条同数值移植：
//   R1 距离度量 22 断言 / R2 只抑制不放大 10 断言 / R3 采集时效 6 断言 = 38。
//   R4 指令仲裁 15 断言测的是 bridge_core.CmdArbiter（桥侧 Python），
//   不在 veto 移植范围，继续由 Python 套件覆盖（53 总数 = 38 + 15 对账）。
// 与 Python 的唯一已知偏差：R2 性质测试的随机样本流——numpy default_rng(42)
// 无法在 C++ 复现，改用 mt19937_64 固定种子；性质本身（|out|≤|in| 且
// PASS=原样）与样本无关，结论等价。
// 运行: ros2 run obstacle_veto_cpp veto_core_test   （依赖仅 Eigen；失败非零退出）
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "obstacle_veto_cpp/veto_core.hpp"

using veto_core::Geom;
using veto_core::Params;
using veto_core::SL;
using veto_core::prior_n;
using veto_core::PRIOR_H;

static int n_checks = 0;
static int n_fails = 0;
static std::vector<std::string> fails;

static void check(const std::string& name, bool cond, const std::string& detail = "") {
    ++n_checks;
    if (cond) {
        std::printf("  [PASS] %s\n", name.c_str());
    } else {
        ++n_fails;
        fails.push_back(name);
        std::printf("  [FAIL] %s%s%s\n", name.c_str(),
                    detail.empty() ? "" : " — ", detail.c_str());
    }
}

static bool approx(double a, double b, double tol = 1e-6) {
    return std::fabs(a - b) <= tol;
}

static std::string sl_str(const std::vector<SL>& sl) {
    std::string s = "[";
    char buf[64];
    for (size_t i = 0; i < sl.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s(%.6f, %.6f)",
                     i ? ", " : "", sl[i].dh, sl[i].az);
        s += buf;
    }
    return s + "]";
}

// ---------------- R1：距离度量 ----------------
static void test_r1_geometry() {
    std::printf("R1 距离度量（倾斜系投影压缩 bug）\n");
    Params prm;
    Geom g(prior_n(), PRIOR_H, 0.05);

    // 评审复现：狗身轴心前方 1.20m、离地 0.40m 的单点（轴心在传感器投影前
    // 0.05m）→ 旧公式 0.8162559（压缩 ×0.6802），正确值 1.20
    Eigen::MatrixX3d P(1, 3);
    P.row(0) = g.to_sensor(1.25, 0.0, 0.40);
    auto sl = veto_core::ingest(P, g, prm);
    check("评审复现: 真前向1.20m→dh=1.20",
          sl.size() == 1 && approx(sl[0].dh, 1.20), "got " + sl_str(sl));
    check("评审复现: az=0(正前)", sl.size() == 1 && approx(sl[0].az, 0.0));

    // 各向同性：同一水平距离 1.0m（距轴心）、不同方位 → dh 必须同为 1.0。
    // 旧公式前向 0.68、侧向 1.0（椭圆），这是最直接的回归哨兵
    const int azs[] = {0, 30, 59, 61, 90, 150, 180, 270};
    for (int az_deg : azs) {
        const double azr = az_deg * M_PI / 180.0;
        P.row(0) = g.to_sensor(0.05 + std::cos(azr), std::sin(azr), 0.50);
        sl = veto_core::ingest(P, g, prm);
        char name[48];
        std::snprintf(name, sizeof(name), "方位 %3d°: dh=1.00", az_deg);
        check(name, sl.size() == 1 && approx(sl[0].dh, 1.0, 1e-5),
              "got " + sl_str(sl));
    }

    // 多倾角：0°平装 / 25° / 47°先验 / 30°侧倾（绕x轴）——真距离与倾角无关
    struct TiltCase { const char* name; Eigen::Vector3d n; };
    const double s25 = std::sin(25.0 * M_PI / 180.0);
    const double s30 = std::sin(30.0 * M_PI / 180.0);
    std::vector<TiltCase> geoms = {
        {"平装0°", Eigen::Vector3d(0.0, 0.0, 1.0)},
        {"前倾25°", Eigen::Vector3d(-s25, 0.0, std::cos(25.0 * M_PI / 180.0))},
        {"前倾47°(先验)", prior_n()},
        {"侧倾30°", Eigen::Vector3d(0.0, -s30, std::cos(30.0 * M_PI / 180.0))},
    };
    for (const auto& tc : geoms) {
        Geom gg(tc.n, 0.486, 0.05);
        P.row(0) = gg.to_sensor(1.25, 0.0, 0.40);
        sl = veto_core::ingest(P, gg, prm);
        std::string name = std::string("倾角") + tc.name + ": 前向1.20m→dh=1.20";
        check(name, sl.size() == 1 && approx(sl[0].dh, 1.20, 1e-5),
              "got " + sl_str(sl));
    }

    // 方位符号：+左。狗系 (1.05, +0.5, 0.5) → dh=1.12<near_keep，az>0
    P.row(0) = g.to_sensor(1.05, 0.5, 0.50);
    sl = veto_core::ingest(P, g, prm);
    check("az 符号: 左正", sl.size() == 1 && sl[0].az > 0, "got " + sl_str(sl));

    // 频带外剔除：h=0.10（<band_lo）/ h=0.80（>band_hi）
    P.resize(2, 3);
    P.row(0) = g.to_sensor(1.25, 0.0, 0.10);
    P.row(1) = g.to_sensor(1.25, 0.0, 0.80);
    sl = veto_core::ingest(P, g, prm);
    check("频带外(0.10m/0.80m高)剔除", sl.empty(),
          "got " + std::to_string(sl.size()) + " 点");

    // 自体掩膜：传感器正下方（狗系 0,0,0.45 → 传感器系 z≈-0.036 落掩膜盒）
    P.resize(1, 3);
    P.row(0) = g.to_sensor(0.0, 0.0, 0.45);
    sl = veto_core::ingest(P, g, prm);
    check("自体掩膜剔除", sl.empty(),
          "got " + std::to_string(sl.size()) + " 点");

    // near_keep：2.0m 外不入库
    P.row(0) = g.to_sensor(2.0, 0.0, 0.50);
    sl = veto_core::ingest(P, g, prm);
    check("near_keep=1.25: 2.0m外不入库", sl.empty(),
          "got " + std::to_string(sl.size()) + " 点");

    // 半径方位语义：前扇区远、侧后环近；±60° 恰在边界算环（严格小于）
    check("半径: az=59°→sector_r",
          approx(veto_core::radius(59.0 * M_PI / 180.0, prm), 1.0));
    check("半径: az=61°→ring_r",
          approx(veto_core::radius(61.0 * M_PI / 180.0, prm), 0.55));
    check("半径: az=60°边界→ring_r",
          approx(veto_core::radius(60.0 * M_PI / 180.0, prm), 0.55));

    // Geom.rebase 保体轴心（IMU 增量跟踪后 body_c 仍沿新前向 +0.05）
    Geom gg(prior_n(), 0.486, 0.05);
    gg.rebase(Eigen::Vector3d(-0.5, 0.3, 0.8));
    check("rebase: body_c 沿新前向 0.05",
          approx(gg.body_c.dot(gg.f), 0.05) && approx(gg.body_c.dot(gg.l), 0.0));
}

// ---------------- R2：只抑制不放大 ----------------
static void test_r2_no_amplify() {
    std::printf("R2 只抑制不放大（残速抬升越权已删）\n");
    Params prm;

    // 空场放行：0.05 原样（起步死区是上游 speed_adapter 的职责，veto 不越权）
    auto r = veto_core::apply_veto(0.05, 0.0, {}, prm);
    check("空场: 0.05 原样放行",
          r.state == veto_core::State::PASS && approx(r.vx, 0.05) && approx(r.vy, 0));

    // 未消解指令原样：侧向障碍、前进指令无朝向分量 → 不动它（哪怕 <地板）
    {
        std::vector<SL> pts = {{0.5, 90.0 * M_PI / 180.0}};
        r = veto_core::apply_veto(0.10, 0.0, pts, prm);
        char det[64];
        std::snprintf(det, sizeof(det), "got (%.3f,%.3f)", r.vx, r.vy);
        check("侧障碍+前进: 未消解→原样",
              r.state == veto_core::State::BLOCK_RING && approx(r.vx, 0.10) &&
              approx(r.vy, 0), det);
    }

    // 正前拦截：朝向分量全消
    {
        std::vector<SL> pts = {{0.6, 0.0}};
        r = veto_core::apply_veto(0.25, 0.0, pts, prm);
        check("正前0.6m: 全消→(0,0)",
              r.state == veto_core::State::BLOCK_SECTOR && r.vx == 0 && r.vy == 0);
        check("blk 回报 (dh,az°)",
              r.has_blk && approx(r.blk_dh, 0.6) && approx(r.blk_az_deg, 0));

        // ★ R2 回归哨兵：45° 障碍 + 0.25 输入 → 残速 0.177<地板 → 归零。
        // 旧版此处抬升成 (0, 0.25)——输出是输入的 √2 倍，越权必须死
        pts = {{0.6, 45.0 * M_PI / 180.0}};
        r = veto_core::apply_veto(0.25, 0.0, pts, prm);
        char det[80];
        std::snprintf(det, sizeof(det), "got (%.4f,%.4f) — 旧bug会是 (0,0.25)",
                      r.vx, r.vy);
        check("45°障碍+0.25: 残速<地板→归零(不抬升)",
              r.vx == 0 && r.vy == 0, det);

        // 45° 障碍 + 0.40 输入 → 消解朝向分量后残速 0.283≥地板 → 滑行。
        // 残差=输入减去(投影到障碍方向)→与障碍方向正交: (0.2,-0.2)，不放大
        r = veto_core::apply_veto(0.40, 0.0, pts, prm);
        std::snprintf(det, sizeof(det), "got (%.4f,%.4f)", r.vx, r.vy);
        check("45°障碍+0.40: 残速≥地板→滑行",
              approx(r.vx, 0.2, 1e-9) && approx(r.vy, -0.2, 1e-9), det);
        check("滑行不放大: |out|≤|in|", std::hypot(r.vx, r.vy) <= 0.40 + 1e-9);
    }

    // 极近全消：0.25m<touch_r，哪怕纯侧向速度也归零
    {
        std::vector<SL> pts = {{0.25, 0.0}};
        r = veto_core::apply_veto(0.0, 0.30, pts, prm);
        check("极近0.25m: 全消平动", r.vx == 0 && r.vy == 0);
    }

    // 身后障碍不拦前进（脱离方向永远放行）
    {
        std::vector<SL> pts = {{0.4, M_PI}};
        r = veto_core::apply_veto(0.30, 0.0, pts, prm);
        char det[64];
        std::snprintf(det, sizeof(det), "got (%.3f,%.3f)", r.vx, r.vy);
        check("身后障碍: 前进放行", approx(r.vx, 0.30) && approx(r.vy, 0), det);
    }

    // 性质测试（固定种子）：任意输入×任意障碍组，输出模长永不增。
    // 样本流与 numpy default_rng(42) 不同（mt19937_64），性质等价
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    bool ok = true;
    std::string bad;
    for (int it = 0; it < 2000 && ok; ++it) {
        const double va = uni(rng) * 2.0 * M_PI;
        const double vs = uni(rng) * 0.6;
        const double vinx = vs * std::cos(va), viny = vs * std::sin(va);
        const int nb = static_cast<int>(uni(rng) * 4.0);  // 0..3
        std::vector<SL> blockers;
        blockers.reserve(nb);
        for (int j = 0; j < nb; ++j) {
            blockers.push_back({0.15 + uni(rng) * (1.4 - 0.15),
                                 -M_PI + uni(rng) * 2.0 * M_PI});
        }
        auto rr = veto_core::apply_veto(vinx, viny, blockers, prm);
        if (std::hypot(rr.vx, rr.vy) > std::hypot(vinx, viny) + 1e-9) {
            ok = false;
            bad = "放大 @" + std::to_string(it);
        } else if (rr.state == veto_core::State::PASS &&
                   (!approx(rr.vx, vinx, 1e-9) || !approx(rr.vy, viny, 1e-9))) {
            ok = false;
            bad = "PASS被改动 @" + std::to_string(it);
        }
    }
    check("性质: 2000组随机输入 |out|≤|in| 且 PASS=原样", ok, bad);
}

// ---------------- R3：采集时效 ----------------
static void test_r3_stamp() {
    std::printf("R3 采集时效（刚收到≠刚采集）\n");
    Params prm;
    auto sr = veto_core::stamp_ok(100.0, true, 99.9, 100.15, prm);
    check("新鲜前进: 放行", sr.ok && sr.reason == "ok");
    sr = veto_core::stamp_ok(100.0, false, 0.0, 100.20, prm);
    check("首帧新鲜: 放行", sr.ok);
    sr = veto_core::stamp_ok(100.0, false, 0.0, 103.0, prm);
    check("超龄 3.0s>2.5: 拒('age')", !sr.ok && sr.reason == "age");
    sr = veto_core::stamp_ok(100.0, true, 100.0, 100.1, prm);
    check("重复戳(冻结): 拒('frozen')", !sr.ok && sr.reason == "frozen");
    sr = veto_core::stamp_ok(99.5, true, 100.0, 100.1, prm);
    check("戳回退: 拒('frozen')", !sr.ok && sr.reason == "frozen");
    Params prm_off;
    prm_off.stamp_check = false;
    sr = veto_core::stamp_ok(100.0, true, 100.0, 999.0, prm_off);
    check("闸门关闭(回放模式): 放行", sr.ok && sr.reason == "off");
}

int main() {
    test_r1_geometry();
    test_r2_no_amplify();
    test_r3_stamp();
    std::printf("\n==== 回归汇总: %d 项断言, 失败 %d ====\n", n_checks, n_fails);
    if (n_fails > 0) {
        std::string names;
        for (const auto& f : fails) names += (names.empty() ? "" : ", ") + f;
        std::printf("失败项: %s\n", names.c_str());
        return 1;
    }
    std::printf("全部通过 ✓（R1-R3；R4 仲裁 15 断言在 bridge 侧 Python 套件）\n");
    return 0;
}
