#include "Scenes.h"
#include <cmath>
#include <random>

const char* sceneName(SceneId id) {
    switch (id) {
        case SceneId::Room:      return "Room";
        case SceneId::Tunnel:    return "Tunnel";
        case SceneId::MultiRate: return "Multi-rate";
        case SceneId::Stack:     return "Stack";
    }
    return "?";
}

static Body makeBody(Vec2 pos, Vec2 vel, float r, bool gravity = true) {
    Body b;
    b.pos = b.prevStep = b.prevFrame = pos;
    b.vel = vel;
    b.radius = r;
    b.invMass = invMassOfRadius(r);
    b.useGravity = gravity;
    return b;
}

static Body makeStatic(Vec2 pos, float r) {
    Body b = makeBody(pos, Vec2(), r, false);
    b.invMass = 0.f;
    return b;
}

// Комната: четыре толстые стенки СНАРУЖИ окна (чтобы даже быстрый шар не проскочил наружу).
static void addRoom(World& w) {
    const float W = (float)cfg::WindowW, H = (float)cfg::WindowH, T = cfg::WallThickness;
    w.walls.push_back({Vec2(-T, H),  Vec2(W + T, H + T), false});   // пол
    w.walls.push_back({Vec2(-T, -T), Vec2(W + T, 0.f),   false});   // потолок
    w.walls.push_back({Vec2(-T, -T), Vec2(0.f, H + T),   false});   // левая
    w.walls.push_back({Vec2(W, -T),  Vec2(W + T, H + T), false});   // правая
}

static void addThinWall(World& w) {
    const float H = (float)cfg::WindowH;
    Wall t{Vec2(cfg::ThinWallX0, 0.f), Vec2(cfg::ThinWallX0 + cfg::ThinWallThickness, H), true};
    w.walls.push_back(t);
    w.thinWallIndex = (int)w.walls.size() - 1;
}

// Детерминированный генератор: mt19937 + свой перевод в [0,1) (не зависит от стандартной библиотеки).
struct Rng {
    std::mt19937 g;
    explicit Rng(unsigned seed) : g(seed) {}
    float next01() { return (float)(g() >> 8) * (1.0f / 16777216.0f); }
    float range(float a, float b) { return a + (b - a) * next01(); }
};

static void buildRoom(World& w) {
    addRoom(w);
    for (int i = 0; i < cfg::RoomBalls; ++i) {
        float r = cfg::RoomRadiusMin + i * cfg::RoomRadiusStep;
        float sgn = (i % 2) ? -1.f : 1.f;
        w.slow.push_back(makeBody(Vec2(200.f + i * 220.f, 120.f + (i % 2) * 80.f),
                                  Vec2(sgn * (150.f + 40.f * i), 0.f), r));
    }
}

static void buildTunnel(World& w, const SceneParams& p) {
    addRoom(w);
    addThinWall(w);
    const float s = p.projectileSpeed * cfg::TunnelPhaseDt;           // путь за шаг 1/60
    for (int i = 0; i < cfg::TunnelProjectiles; ++i) {
        // x_i = X0 + i * v*dt / N : каждый следующий снаряд сдвинут на 1/N шага (разные фазы)
        float x = cfg::TunnelX0 + i * s / (float)cfg::TunnelProjectiles;
        float y = cfg::TunnelY0 + i * cfg::TunnelYStep;
        w.fast.push_back(makeBody(Vec2(x, y), Vec2(p.projectileSpeed, 0.f), cfg::ProjectileRadius, false));
    }
}

static bool freeSpot(const World& w, Vec2 pos, float r, float margin) {
    for (const Body& s : w.statics)
        if (length(pos - s.pos) < r + s.radius + margin) return false;
    for (const Body& b : w.slow)
        if (length(pos - b.pos) < r + b.radius + margin) return false;
    return true;
}

static void buildMultiRate(World& w, const SceneParams& p) {
    addRoom(w);
    const float W = (float)cfg::WindowW;
    Rng rng(cfg::Seed);
    // неподвижные круги: равномерно по x, чётные/нечётные сдвинуты по y
    const float span = W - 2.f * cfg::StaticMarginX;
    for (int i = 0; i < cfg::StaticCount; ++i) {
        float x = cfg::StaticMarginX + (i + 0.5f) * span / (float)cfg::StaticCount;
        float y = cfg::StaticY0 + (i % 2) * cfg::StaticYAlt;
        w.statics.push_back(makeStatic(Vec2(x, y), cfg::StaticRadius));
    }
    // обычные шары: случайные размеры и позиции (seed из Config), без перекрытий
    for (int i = 0; i < cfg::SlowCount; ++i) {
        for (int attempt = 0; attempt < 500; ++attempt) {
            float r = rng.range(cfg::SlowRadiusMin, cfg::SlowRadiusMax);
            Vec2 pos(rng.range(60.f, W - 60.f), rng.range(cfg::SlowAreaYMin, cfg::SlowAreaYMax));
            Vec2 vel(rng.range(-cfg::SlowSpeedMaxX, cfg::SlowSpeedMaxX),
                     rng.range(-cfg::SlowSpeedMaxY, cfg::SlowSpeedMaxY));
            if (freeSpot(w, pos, r, 2.f)) { w.slow.push_back(makeBody(pos, vel, r)); break; }
        }
    }
    // быстрые снаряды: формула, без случайности, гравитация выключена
    for (int i = 0; i < cfg::FastCount; ++i) {
        Vec2 pos(cfg::FastX0, cfg::FastY0 + i * cfg::FastYStep);
        Vec2 vel(p.projectileSpeed, (i - cfg::FastCount / 2) * cfg::FastVyStep);
        w.fast.push_back(makeBody(pos, vel, cfg::ProjectileRadius, false));
    }
}

static void buildStack(World& w) {
    addRoom(w);
    const float W = (float)cfg::WindowW, H = (float)cfg::WindowH;
    const float r = cfg::StackRadius;
    const float dx = 2.f * r + cfg::StackGap;
    const float dy = dx * 0.8660254f;               // высота ряда гексагональной укладки
    const float cx = W * 0.5f;
    for (int k = 0; k < cfg::StackRows; ++k) {
        int n = cfg::StackRows - k;
        for (int j = 0; j < n; ++j) {
            float x = cx + (j - (n - 1) * 0.5f) * dx;
            float y = H - r - cfg::StackGap - k * dy;
            w.slow.push_back(makeBody(Vec2(x, y), Vec2(), r));
        }
    }
    // упоры по бокам нижнего ряда, чтобы он не разъезжался
    const float half = (cfg::StackRows - 1) * 0.5f * dx + r + cfg::StackGap;
    w.walls.push_back({Vec2(cx - half - 40.f, H - 3.f * cfg::StackRadius * cfg::StackRows), Vec2(cx - half, H), false});
    w.walls.push_back({Vec2(cx + half, H - 3.f * cfg::StackRadius * cfg::StackRows), Vec2(cx + half + 40.f, H), false});
}

void buildScene(World& w, SceneId id, const SceneParams& p) {
    w = World();
    w.restitution = p.restitution;
    switch (id) {
        case SceneId::Room:      buildRoom(w); break;
        case SceneId::Tunnel:    buildTunnel(w, p); break;
        case SceneId::MultiRate: buildMultiRate(w, p); break;
        case SceneId::Stack:     buildStack(w); break;
    }
}
