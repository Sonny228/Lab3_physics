#include "Experiments.h"
#include "Scenes.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>

namespace {

struct Out {
    std::ofstream f;
    explicit Out(const std::string& path) : f(path) {}
    void line(const std::string& s = "") { std::cout << s << '\n'; if (f) f << s << '\n'; }
};

std::string fmt(const char* format, ...) {
    char buf[1024];
    va_list ap; va_start(ap, format);
    std::vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    return buf;
}

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

float theoryP(float v, float stepPerSecond /*частота шага*/) {
    float s = v / stepPerSecond;
    float band = cfg::ThinWallThickness + 2.f * cfg::ProjectileRadius;
    return std::max(0.f, 1.f - band / s);
}

// ---------- туннелирование: один снаряд, заданная фаза ----------
bool tunnelOnce(Mode mode, int baseHz, float speed, int phase, int phases) {
    SceneParams sp; sp.projectileSpeed = speed;
    World w; buildScene(w, SceneId::Tunnel, sp);
    // эффективный шаг, который определяет туннелирование
    float stepHz = (float)baseHz * (mode == Mode::MultiRate ? (float)cfg::FastSubstepsK : 1.f);
    float s = speed / stepHz;
    float bandStart = cfg::ThinWallX0 - cfg::ProjectileRadius;
    int K = std::max(1, (int)((bandStart - 60.f) / s));
    float x0 = bandStart + (phase + 0.5f) / (float)phases * s - K * s;
    Body b = w.fast[0];
    b.pos = b.prevStep = b.prevFrame = Vec2(x0, 360.f);
    b.vel = Vec2(speed, 0.f);
    w.fast.clear(); w.fast.push_back(b);
    StepStats st;
    int maxSteps = 3 * baseHz;
    for (int i = 0; i < maxSteps; ++i) {
        stepWorld(w, mode, 1.f / (float)baseHz, st);
        if (w.fast[0].hitThin) return false;
        if (w.fast[0].tunneled) return true;
    }
    return false;
}

int tunnelPhases(Mode mode, int baseHz, float speed) {
    int n = 0;
    for (int p = 0; p < cfg::TunnelPhases; ++p) n += tunnelOnce(mode, baseHz, speed, p, cfg::TunnelPhases) ? 1 : 0;
    return n;
}

int tunnelScene(Mode mode, int baseHz) {
    World w; buildScene(w, SceneId::Tunnel);
    StepStats st;
    for (int i = 0; i < 2 * baseHz; ++i) {
        stepWorld(w, mode, 1.f / (float)baseHz, st);
        bool done = true;
        for (const Body& b : w.fast) if (!b.hitThin && !b.tunneled) done = false;
        if (done) break;
    }
    return w.tunneledCount();
}

// ---------- детерминированность ----------
uint64_t hashWorld(const World& w) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](float f) {
        uint32_t u; std::memcpy(&u, &f, 4);
        for (int i = 0; i < 4; ++i) { h ^= (uint64_t)((u >> (8 * i)) & 0xFFu); h *= 1099511628211ull; }
    };
    auto body = [&](const Body& b) { mix(b.pos.x); mix(b.pos.y); mix(b.vel.x); mix(b.vel.y); };
    for (const Body& b : w.slow) body(b);
    for (const Body& b : w.fast) body(b);
    return h;
}

uint64_t detRun(Mode mode, bool floating, unsigned dtSeed) {
    World w; buildScene(w, SceneId::MultiRate);
    StepStats st;
    const int hz = cfg::DefaultHz;
    if (!floating) {
        int steps = (int)(cfg::DeterminismSecs * hz);
        for (int i = 0; i < steps; ++i) stepWorld(w, mode, 1.f / (float)hz, st);
    } else {
        std::mt19937 g(dtSeed);
        double t = 0.0;
        while (t < cfg::DeterminismSecs) {
            double u = (double)(g() >> 8) / 16777216.0;
            double dt = cfg::FloatDtMin + (cfg::FloatDtMax - cfg::FloatDtMin) * u;
            stepWorld(w, mode, (float)dt, st);
            t += dt;
        }
    }
    return hashWorld(w);
}

// ---------- время шага ----------
struct TimeRes { double msPerStep, contacts, limitShare; };
TimeRes timeRun(SceneId sc, Mode m, int hz, double secs) {
    World w; buildScene(w, sc);
    StepStats st;
    int steps = (int)(secs * hz);
    auto t0 = Clock::now();
    for (int i = 0; i < steps; ++i) stepWorld(w, m, 1.f / (float)hz, st);
    double ms = msSince(t0);
    TimeRes r;
    r.msPerStep = ms / steps;
    r.contacts = (double)st.contacts / steps;
    r.limitShare = (double)st.ccdLimitHits / steps;
    return r;
}

// ---------- стек ----------
struct StackRes { float sink, meanV, maxPen; bool stands; };
StackRes stackRun(Mode m, int hz) {
    World w; buildScene(w, SceneId::Stack);
    StepStats st;
    int total = (int)(cfg::StackSecs * hz);
    int measure = (int)(cfg::StackMeasureSecs * hz);
    float topY0 = w.slow.back().pos.y;
    double sumV = 0.0; int samples = 0; float maxPen = 0.f;
    for (int i = 0; i < total; ++i) {
        stepWorld(w, m, 1.f / (float)hz, st);
        if (i >= total - measure) {
            double v = 0.0;
            for (const Body& b : w.slow) v += length(b.vel);
            sumV += v / (double)w.slow.size(); ++samples;
            for (std::size_t a = 0; a < w.slow.size(); ++a)
                for (std::size_t c = a + 1; c < w.slow.size(); ++c) {
                    float pen = w.slow[a].radius + w.slow[c].radius - length(w.slow[a].pos - w.slow[c].pos);
                    maxPen = std::max(maxPen, pen);
                }
        }
    }
    StackRes r;
    r.sink = w.slow.back().pos.y - topY0;
    r.meanV = samples ? (float)(sumV / samples) : 0.f;
    r.maxPen = maxPen;
    r.stands = r.sink < cfg::StackStandSink && r.meanV < cfg::StackStandSpeed;
    return r;
}

const char* hzName(int hz, Mode m, char* buf) {
    if (m == Mode::MultiRate) std::snprintf(buf, 32, "%d/%d", hz, hz * cfg::FastSubstepsK);
    else std::snprintf(buf, 32, "%d", hz);
    return buf;
}

} // namespace

void runAllExperiments(const std::string& outPath) {
    Out o(outPath);
    char nb[32];
    const Mode modes[3] = {Mode::Discrete, Mode::CCD, Mode::MultiRate};
    const float v = cfg::ProjectileSpeed, r = cfg::ProjectileRadius, wt = cfg::ThinWallThickness;
    const float dt = 1.f / (float)cfg::DefaultHz;

    o.line("==== PZ3 experiments (Release build recommended) ====");
    o.line(fmt("v = %.0f px/s, thin wall = %.0f px, r = %.0f px, e = %.2f, g = %.0f, dt = 1/%d, k = %d, seed = %u",
               v, wt, r, cfg::Restitution, cfg::Gravity, cfg::DefaultHz, cfg::FastSubstepsK, cfg::Seed));
    o.line(fmt("scene Multi-rate: %d static (r=%.0f), %d slow (r=%.0f..%.0f), %d fast (r=%.0f); stack: %d balls r=%.0f",
               cfg::StaticCount, cfg::StaticRadius, cfg::SlowCount, cfg::SlowRadiusMin, cfg::SlowRadiusMax,
               cfg::FastCount, r, cfg::StackRows * (cfg::StackRows + 1) / 2, cfg::StackRadius));
    auto warn = cfg::validate();
    if (warn.empty()) o.line("Config vs TZ: OK");
    for (auto& s : warn) o.line("CONFIG WARNING: " + s);

    // ---- A. примеры расчёта для отчёта ----
    o.line(); o.line("---- A. Worked examples (for report sections 1.2-1.5) ----");
    {
        float v0 = -cfg::ExampleThrowSpeed;
        float v1 = v0 + cfg::Gravity * dt, y1 = cfg::ExampleY0 + v1 * dt;
        o.line(fmt("(1)(2) semi-implicit Euler: v1 = %.2f + %.0f*%.5f = %.2f px/s, y1 = %.2f + (%.2f)*%.5f = %.2f px",
                   v0, cfg::Gravity, dt, v1, cfg::ExampleY0, v1, dt, y1));
        o.line("(3) steps per frame N = f/FPS at reference FPS " + fmt("%.0f", cfg::ReferenceFps) + ":");
        for (int f : cfg::FreqPresets)
            o.line(fmt("    f = %3d Hz: normal %.3f, fast (k=%d) %.3f steps/frame", f, f / cfg::ReferenceFps,
                       cfg::FastSubstepsK, cfg::FastSubstepsK * f / cfg::ReferenceFps));
        float s = v * dt;
        o.line(fmt("(9) s = v*dt = %.0f * %.5f = %.2f px per 60 Hz step; band w+2r = %.0f px", v, dt, s, wt + 2 * r));
        o.line(fmt("(10) P = 1 - %.0f/%.2f = %.3f", wt + 2 * r, s, theoryP(v, (float)cfg::DefaultHz)));
        float dtf = dt / cfg::FastSubstepsK;
        o.line(fmt("(16) dt_fast = dt/k = %.6f s, path per substep = %.2f px", dtf, v * dtf));
        o.line(fmt("(17) v* = (w+2r)/dt_fast = %.0f px/s (multi-rate stops protecting above this speed)", (wt + 2 * r) / dtf));

        // CCD: верхний снаряд сцены Tunnel
        float x0 = cfg::TunnelX0;
        float band = cfg::ThinWallX0 - r;
        int n = (int)std::floor((band - x0) / s);
        float xn = x0 + n * s, tc = (band - xn) / v;
        float vAfter = -cfg::Restitution * v, rem = dt - tc;
        o.line(fmt("(15) top projectile: step #%d starts at x0 = %.2f, wall face %.0f: tc = (%.0f - %.0f - %.2f)/%.0f = %.5f s = %.2f ms (%.1f%% of %.2f ms step)",
                   n + 1, xn, cfg::ThinWallX0, cfg::ThinWallX0, r, xn, v, tc, tc * 1000, tc / dt * 100, dt * 1000));
        o.line(fmt("     after hit v = -e*v = %.0f px/s, rest of step %.2f ms, x_end(CCD) = %.2f px; without CCD x_end = %.2f px",
                   vAfter, rem * 1000, band + vAfter * rem, xn + s));
        // квадратное уравнение
        float d = cfg::ExampleCircleDist, r2 = cfg::SlowRadiusMax, R = r + r2;
        double a = (double)v * v, b = 2.0 * (-d) * v, c = (double)d * d - (double)R * R;
        double D = b * b - 4 * a * c, tq = (-b - std::sqrt(D)) / (2 * a);
        o.line(fmt("(12)-(14) projectile r=%.0f at 0 vs ball r=%.0f at %.0f: a=%.4g, b=%.4g, c=%.4g, D=%.4g, tc=%.5f s (check: (%.0f-%.0f)/%.0f = %.5f)",
                   r, r2, d, a, b, c, D, tq, d, R, v, (d - R) / v));
        // импульс
        float m1 = cfg::MassScale * r * r, m2 = cfg::MassScale * r2 * r2, e = cfg::Restitution;
        float j = (1 + e) * v / (1.f / m1 + 1.f / m2);
        float u1 = v - j / m1, u2 = j / m2;
        o.line(fmt("(8) head-on hit on resting ball: m1=%.0f, m2=%.0f, j = %.0f, v1' = %.1f, v2' = %.1f; momentum %.0f -> %.0f; separation speed %.1f (= e*v = %.1f)",
                   m1, m2, j, u1, u2, m1 * v, m1 * u1 + m2 * u2, u2 - u1, e * v));
        o.line(fmt("(7) wall hit: v_n' = -%.2f * %.0f = %.0f px/s", e, v, -e * v));
    }

    // ---- B. Туннелирование ----
    o.line(); o.line("---- B. Tunneling (phases = " + std::to_string(cfg::TunnelPhases) + ") ----");
    o.line("B1. Discrete, v = " + fmt("%.0f", v) + " px/s, by physics rate:");
    o.line("  f, Hz | s per step, px | theory P, % | measured, % | delta");
    for (int f : cfg::FreqPresets) {
        float th = theoryP(v, (float)f) * 100.f;
        int m = tunnelPhases(Mode::Discrete, f, v);
        o.line(fmt("  %5d | %14.2f | %11.1f | %11d | %.1f", f, v / f, th, m, std::fabs(th - m)));
    }
    int sc = tunnelScene(Mode::Discrete, cfg::DefaultHz);
    o.line(fmt("  scene Tunnel (%d projectiles, Discrete 60 Hz): theory %d of %d, measured %d of %d",
               cfg::TunnelProjectiles, (int)std::lround(theoryP(v, (float)cfg::DefaultHz) * cfg::TunnelProjectiles),
               cfg::TunnelProjectiles, sc, cfg::TunnelProjectiles));
    o.line(fmt("  scene Tunnel, CCD 60 Hz: %d of %d;  Multi-rate 60/240: %d of %d",
               tunnelScene(Mode::CCD, cfg::DefaultHz), cfg::TunnelProjectiles,
               tunnelScene(Mode::MultiRate, cfg::DefaultHz), cfg::TunnelProjectiles));
    o.line();
    o.line("B2. Tunneling vs speed, base " + std::to_string(cfg::DefaultHz) + " Hz, % tunneled:");
    o.line("  v, px/s | Discrete 60 (theory / measured) | Multi-rate 240 (theory / measured) | CCD");
    for (float sp : cfg::TunnelSpeeds) {
        float t1 = theoryP(sp, (float)cfg::DefaultHz) * 100.f;
        float t2 = theoryP(sp, (float)cfg::DefaultHz * cfg::FastSubstepsK) * 100.f;
        int m1 = tunnelPhases(Mode::Discrete, cfg::DefaultHz, sp);
        int m2 = tunnelPhases(Mode::MultiRate, cfg::DefaultHz, sp);
        int m3 = tunnelPhases(Mode::CCD, cfg::DefaultHz, sp);
        o.line(fmt("  %7.0f | %14.1f / %-14d | %16.1f / %-16d | %d", sp, t1, m1, t2, m2, m3));
    }

    // ---- C. Детерминированность ----
    o.line(); o.line("---- C. Determinism: FNV-1a hash of all positions and velocities after "
                     + fmt("%.0f", cfg::DeterminismSecs) + " s, scene Multi-rate ----");
    o.line("  mode       | fixed dt run 1   | fixed dt run 2   | same | float dt run A   | float dt run B   | same");
    for (Mode m : modes) {
        uint64_t f1 = detRun(m, false, 0), f2 = detRun(m, false, 0);
        uint64_t g1 = detRun(m, true, cfg::FloatDtSeedA), g2 = detRun(m, true, cfg::FloatDtSeedB);
        o.line(fmt("  %-10s | %016llx | %016llx | %-4s | %016llx | %016llx | %s", modeName(m),
                   (unsigned long long)f1, (unsigned long long)f2, f1 == f2 ? "yes" : "NO",
                   (unsigned long long)g1, (unsigned long long)g2, g1 == g2 ? "yes" : "no"));
    }

    // ---- D. Время шага ----
    o.line(); o.line("---- D. Step time, scene Multi-rate, " + fmt("%.0f", cfg::TimingSecs) + " s of simulation ----");
    o.line(fmt("  ms/frame is computed for FPS = %.0f (Config::ReferenceFps): ms/frame = ms_per_step * f / FPS", cfg::ReferenceFps));
    o.line("  mode       | f, Hz   | ms/step | ms per 1 s sim | ms/frame | contacts/step | CCD limit hits, %");
    for (Mode m : modes)
        for (int f : cfg::FreqPresets) {
            TimeRes t = timeRun(SceneId::MultiRate, m, f, cfg::TimingSecs);
            o.line(fmt("  %-10s | %-7s | %7.4f | %14.3f | %8.4f | %13.1f | %.1f", modeName(m), hzName(f, m, nb),
                       t.msPerStep, t.msPerStep * f, t.msPerStep * f / cfg::ReferenceFps, t.contacts, t.limitShare * 100));
        }

    // ---- E. Стек ----
    o.line(); o.line(fmt("---- E. Stack stability: %d balls r=%.0f, %.0f s, last %.0f s measured ----",
                         cfg::StackRows * (cfg::StackRows + 1) / 2, cfg::StackRadius, cfg::StackSecs, cfg::StackMeasureSecs));
    o.line(fmt("  'stands' = sink < %.0f px and mean |v| < %.0f px/s", cfg::StackStandSink, cfg::StackStandSpeed));
    o.line("  mode       | f, Hz | top sink, px | mean |v|, px/s | max penetration, px | result");
    for (Mode m : modes)
        for (int f : cfg::FreqPresets) {
            StackRes s = stackRun(m, f);
            o.line(fmt("  %-10s | %5d | %12.2f | %13.1f | %19.2f | %s", modeName(m), f, s.sink, s.meanV, s.maxPen,
                       s.stands ? "stands" : "collapsed"));
        }
    o.line(); o.line("==== done, saved to " + outPath + " ====");
}
