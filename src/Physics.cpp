#include "Physics.h"
#include <algorithm>
#include <cfloat>
#include <cmath>

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Discrete:  return "Discrete";
        case Mode::CCD:       return "CCD";
        case Mode::MultiRate: return "Multi-rate";
    }
    return "?";
}

int World::tunneledCount() const {
    int n = 0;
    for (const Body& b : fast) if (b.tunneled) ++n;
    return n;
}

void beginMacroStep(World& w) {
    for (Body& b : w.slow) b.prevFrame = b.pos;
    for (Body& b : w.fast) b.prevFrame = b.pos;
}

// ------------------------------------------------------------------
//  Импульс вдоль нормали n (n направлена от b к a). Формула (8) отчёта:
//  j = -(1+e) * (v_rel . n) / (1/m1 + 1/m2).  Только если тела сближаются.
// ------------------------------------------------------------------
static void applyImpulse(Body& a, Body& b, Vec2 n, float e) {
    float vn = dot(a.vel - b.vel, n);
    if (vn >= 0.f) return;
    float sumInv = a.invMass + b.invMass;
    if (sumInv <= 0.f) return;
    float j = -(1.f + e) * vn / sumInv;
    a.vel += n * (j * a.invMass);
    b.vel -= n * (j * b.invMass);
}

static void updateTunnelFlags(World& w) {
    if (w.thinWallIndex < 0) return;
    float right = w.walls[w.thinWallIndex].max.x;
    for (Body& b : w.fast)
        if (!b.tunneled && !b.hitThin && b.pos.x > right) b.tunneled = true;
}

// ==================================================================
//  ДИСКРЕТНАЯ проверка: шаг, потом поиск перекрытий и выталкивание.
// ==================================================================
static bool resolveCircleWall(Body& b, const Wall& wl, float e, StepStats& st) {
    const float r = b.radius;
    Vec2 closest(std::min(std::max(b.pos.x, wl.min.x), wl.max.x),
                 std::min(std::max(b.pos.y, wl.min.y), wl.max.y));
    Vec2 d = b.pos - closest;
    float d2 = lengthSq(d);
    if (d2 >= r * r) return false;

    // Куда выталкивать. Если в прошлом шаге центр был снаружи с одной стороны
    // (только по x или только по y), выталкиваем на ту же сторону - иначе снаряд
    // с центром глубже середины тонкой стенки вылетал бы на другую сторону (п. 2.1 отчёта).
    Vec2 n; float pen;
    bool outX = b.prevStep.x < wl.min.x || b.prevStep.x > wl.max.x;
    bool outY = b.prevStep.y < wl.min.y || b.prevStep.y > wl.max.y;
    if (outX != outY) {
        if (outX) {
            if (b.prevStep.x < wl.min.x) { n = Vec2(-1.f, 0.f); pen = (b.pos.x + r) - wl.min.x; }
            else                         { n = Vec2( 1.f, 0.f); pen = wl.max.x - (b.pos.x - r); }
        } else {
            if (b.prevStep.y < wl.min.y) { n = Vec2(0.f, -1.f); pen = (b.pos.y + r) - wl.min.y; }
            else                         { n = Vec2(0.f,  1.f); pen = wl.max.y - (b.pos.y - r); }
        }
    } else if (d2 > 1e-12f) {
        float dist = std::sqrt(d2);
        n = d / dist; pen = r - dist;
    } else {  // центр внутри, а по прошлой позиции сторону не определить: минимальное проникновение
        float l = b.pos.x - wl.min.x + r, rr = wl.max.x - b.pos.x + r;
        float t = b.pos.y - wl.min.y + r, bt = wl.max.y - b.pos.y + r;
        float m = std::min(std::min(l, rr), std::min(t, bt));
        if (m == l)       { n = Vec2(-1.f, 0.f); pen = l; }
        else if (m == rr) { n = Vec2( 1.f, 0.f); pen = rr; }
        else if (m == t)  { n = Vec2(0.f, -1.f); pen = t; }
        else              { n = Vec2(0.f,  1.f); pen = bt; }
    }
    if (pen < 0.f) pen = 0.f;
    b.pos += n * pen;
    float vn = dot(b.vel, n);
    if (vn < 0.f) b.vel -= n * ((1.f + e) * vn);   // v_n' = -e * v_n  (формула 7)
    if (wl.thin) b.hitThin = true;
    ++st.contacts;
    return true;
}

static bool resolveCirclePair(Body& a, Body& b, float e, StepStats& st) {
    Vec2 d = a.pos - b.pos;
    float R = a.radius + b.radius;
    float d2 = lengthSq(d);
    if (d2 >= R * R) return false;
    float sumInv = a.invMass + b.invMass;
    if (sumInv <= 0.f) return false;
    float dist = std::sqrt(d2);
    Vec2 n = dist > 1e-6f ? d / dist : Vec2(0.f, 1.f);
    float pen = R - dist;
    a.pos += n * (pen * a.invMass / sumInv);   // растаскиваем пропорционально 1/m
    b.pos -= n * (pen * b.invMass / sumInv);
    applyImpulse(a, b, n, e);
    ++st.contacts;
    return true;
}

// movers шагают; frozen в этом шаге стоят на месте (но могут получить импульс).
static void discreteStep(World& w, std::vector<Body*>& movers, std::vector<Body*>& frozen,
                         float dt, StepStats& st) {
    const float e = w.restitution;
    for (Body* b : movers) {                     // полунеявный Эйлер: сначала v, потом x
        b->prevStep = b->pos;
        if (b->useGravity) b->vel.y += w.gravity * dt;
        b->pos += b->vel * dt;
    }
    for (Body* b : movers) for (const Wall& wl : w.walls) resolveCircleWall(*b, wl, e, st);
    for (Body* b : movers) for (Body& s : w.statics)       resolveCirclePair(*b, s, e, st);
    for (Body* b : movers) for (Body* f : frozen)          resolveCirclePair(*b, *f, e, st);
    for (std::size_t i = 0; i < movers.size(); ++i)
        for (std::size_t j = i + 1; j < movers.size(); ++j)
            resolveCirclePair(*movers[i], *movers[j], e, st);
}

// ==================================================================
//  CCD: ищем самое раннее касание внутри шага, двигаем всё к нему,
//  считаем импульс, повторяем для остатка шага.
// ==================================================================
bool sweepCircleCircle(Vec2 p1, Vec2 v1, float r1, Vec2 p2, Vec2 v2, float r2,
                       float tmax, float& tHit) {
    double dpx = (double)p1.x - p2.x, dpy = (double)p1.y - p2.y;
    double dvx = (double)v1.x - v2.x, dvy = (double)v1.y - v2.y;
    double R = (double)r1 + r2;
    double a = dvx * dvx + dvy * dvy;                    // (13)
    double b = 2.0 * (dpx * dvx + dpy * dvy);
    double c = dpx * dpx + dpy * dpy - R * R;
    if (b >= 0.0) return false;                          // не сближаются
    if (c <= 0.0) { tHit = 0.f; return true; }           // уже касаются
    double D = b * b - 4.0 * a * c;
    if (D < 0.0) return false;                           // пролетают мимо
    double t = (-b - std::sqrt(D)) / (2.0 * a);          // (14): меньший корень
    if (t < 0.0) t = 0.0;
    if (t > tmax) return false;                          // касание будет в следующем шаге
    tHit = (float)t;
    return true;
}

bool sweepCircleAABB(Vec2 p, Vec2 v, float r, const Wall& w, float tmax,
                     float& tHit, Vec2& n) {
    // "раздутый" прямоугольник: центр круга касается стенки, когда входит в него
    const float minx = w.min.x - r, maxx = w.max.x + r;
    const float miny = w.min.y - r, maxy = w.max.y + r;

    if (p.x > minx && p.x < maxx && p.y > miny && p.y < maxy) {   // уже внутри
        float l = p.x - minx, rr = maxx - p.x, t = p.y - miny, bt = maxy - p.y;
        float m = std::min(std::min(l, rr), std::min(t, bt));
        if (m == l) n = Vec2(-1.f, 0.f); else if (m == rr) n = Vec2(1.f, 0.f);
        else if (m == t) n = Vec2(0.f, -1.f); else n = Vec2(0.f, 1.f);
        if (dot(v, n) < 0.f) { tHit = 0.f; return true; }
        return false;
    }
    const float INF = 1e30f;
    float txe = -INF, txx = INF, tye = -INF, tyx = INF;
    if (v.x != 0.f) {
        float a = (minx - p.x) / v.x, b = (maxx - p.x) / v.x;   // (15) отдельно по x
        txe = std::min(a, b); txx = std::max(a, b);
    } else if (p.x <= minx || p.x >= maxx) return false;
    if (v.y != 0.f) {
        float a = (miny - p.y) / v.y, b = (maxy - p.y) / v.y;
        tye = std::min(a, b); tyx = std::max(a, b);
    } else if (p.y <= miny || p.y >= maxy) return false;

    float te = std::max(txe, tye);   // касание = более позднее из двух времён входа
    float tx = std::min(txx, tyx);
    if (te > tx || tx < 0.f) return false;
    if (te < -1e-6f) return false;
    if (te < 0.f) te = 0.f;
    if (te > tmax) return false;
    tHit = te;
    if (txe > tye) n = Vec2(v.x > 0.f ? -1.f : 1.f, 0.f);
    else           n = Vec2(0.f, v.y > 0.f ? -1.f : 1.f);
    return true;
}

namespace {
struct Hit {
    bool found = false;
    float t = FLT_MAX;
    int type = 0;              // 1 - стенка, 2 - круг
    Body* a = nullptr;
    Body* b = nullptr;
    const Wall* wall = nullptr;
    Vec2 n;
};
}

static void ccdStep(World& w, float dt, StepStats& st) {
    std::vector<Body*> dyn;
    dyn.reserve(w.dynamicCount());
    for (Body& b : w.slow) dyn.push_back(&b);
    for (Body& b : w.fast) dyn.push_back(&b);
    const float e = w.restitution;

    for (Body* b : dyn) {                         // скорость по гравитации сразу на весь шаг
        b->prevStep = b->pos;
        if (b->useGravity) b->vel.y += w.gravity * dt;
    }
    float remaining = dt;
    bool limit = true;
    for (int it = 0; it < cfg::MaxCcdIterations; ++it) {
        if (remaining <= 0.f) { limit = false; break; }
        Hit best;
        for (Body* b : dyn) {
            for (const Wall& wl : w.walls) {
                float t; Vec2 n;
                if (sweepCircleAABB(b->pos, b->vel, b->radius, wl, remaining, t, n) && t < best.t) {
                    best.found = true; best.t = t; best.type = 1; best.a = b; best.b = nullptr; best.wall = &wl; best.n = n;
                }
            }
            for (Body& s : w.statics) {
                float t;
                if (sweepCircleCircle(b->pos, b->vel, b->radius, s.pos, Vec2(), s.radius, remaining, t) && t < best.t) {
                    best.found = true; best.t = t; best.type = 2; best.a = b; best.b = &s; best.wall = nullptr;
                }
            }
        }
        for (std::size_t i = 0; i < dyn.size(); ++i)
            for (std::size_t j = i + 1; j < dyn.size(); ++j) {
                float t;
                if (sweepCircleCircle(dyn[i]->pos, dyn[i]->vel, dyn[i]->radius,
                                      dyn[j]->pos, dyn[j]->vel, dyn[j]->radius, remaining, t) && t < best.t) {
                    best.found = true; best.t = t; best.type = 2; best.a = dyn[i]; best.b = dyn[j]; best.wall = nullptr;
                }
            }
        ++st.ccdIterations;
        if (!best.found) { limit = false; break; }

        for (Body* b : dyn) b->pos += b->vel * best.t;      // двигаем все тела к моменту касания
        remaining -= best.t;
        if (best.type == 1) {
            // если из-за округления круг успел чуть зайти в стенку - выталкиваем (только при касании в t = 0)
            const Wall& wl = *best.wall;
            float pen = 0.f;
            if (best.n.x < 0.f)      pen = (best.a->pos.x + best.a->radius) - wl.min.x;
            else if (best.n.x > 0.f) pen = wl.max.x - (best.a->pos.x - best.a->radius);
            else if (best.n.y < 0.f) pen = (best.a->pos.y + best.a->radius) - wl.min.y;
            else                     pen = wl.max.y - (best.a->pos.y - best.a->radius);
            if (pen > 0.f) best.a->pos += best.n * pen;
            float vn = dot(best.a->vel, best.n);
            if (vn < 0.f) best.a->vel -= best.n * ((1.f + e) * vn);
            if (wl.thin) best.a->hitThin = true;
        } else {
            Vec2 d = best.a->pos - best.b->pos;
            float len = length(d);
            Vec2 n = len > 1e-6f ? d / len : Vec2(0.f, 1.f);
            // то же для пары кругов: убираем накопившееся перекрытие (если оно есть)
            float pen = best.a->radius + best.b->radius - len;
            float sumInv = best.a->invMass + best.b->invMass;
            if (pen > 0.f && sumInv > 0.f) {
                best.a->pos += n * (pen * best.a->invMass / sumInv);
                best.b->pos -= n * (pen * best.b->invMass / sumInv);
            }
            applyImpulse(*best.a, *best.b, n, e);
        }
        ++st.contacts;
    }
    if (limit) ++st.ccdLimitHits;                 // остаток шага доезжаем без проверки
    if (remaining > 0.f) for (Body* b : dyn) b->pos += b->vel * remaining;

    if (limit) {
        // Страховка: упёрлись в лимит касаний, перекрытия могли накопиться.
        // Один проход выталкивания только по позициям (скорости не трогаем).
        for (Body* b : dyn)
            for (Body& s : w.statics) {
                Vec2 d = b->pos - s.pos; float R = b->radius + s.radius, len = length(d);
                if (len < R && len > 1e-6f) b->pos += d / len * (R - len);
            }
        for (std::size_t i = 0; i < dyn.size(); ++i)
            for (std::size_t j = i + 1; j < dyn.size(); ++j) {
                Vec2 d = dyn[i]->pos - dyn[j]->pos; float R = dyn[i]->radius + dyn[j]->radius, len = length(d);
                if (len < R && len > 1e-6f) {
                    float sumInv = dyn[i]->invMass + dyn[j]->invMass;
                    float pen = R - len;
                    dyn[i]->pos += d / len * (pen * dyn[i]->invMass / sumInv);
                    dyn[j]->pos -= d / len * (pen * dyn[j]->invMass / sumInv);
                }
            }
    }
}

// ==================================================================
//  MULTI-RATE: быстрые - k подшагов dt/k, потом обычные - один шаг dt.
// ==================================================================
static void multiRateStep(World& w, float dt, int k, StepStats& st) {
    std::vector<Body*> fastP, slowP, none;
    for (Body& b : w.fast) fastP.push_back(&b);
    for (Body& b : w.slow) slowP.push_back(&b);
    const float dtf = dt / (float)k;
    for (int i = 0; i < k; ++i) {                 // обычные стоят на месте, но могут получить импульс
        discreteStep(w, fastP, slowP, dtf, st);
        ++st.fastSteps;
    }
    discreteStep(w, slowP, none, dt, st);
    ++st.slowSteps;
}

void stepWorld(World& w, Mode mode, float dt, StepStats& st, int fastSubsteps) {
    beginMacroStep(w);
    switch (mode) {
        case Mode::Discrete: {                    // все тела одним списком с шагом dt
            std::vector<Body*> all, none;
            for (Body& b : w.slow) all.push_back(&b);
            for (Body& b : w.fast) all.push_back(&b);
            discreteStep(w, all, none, dt, st);
            ++st.slowSteps;
            break;
        }
        case Mode::CCD:
            ccdStep(w, dt, st);
            ++st.slowSteps;
            break;
        case Mode::MultiRate:
            multiRateStep(w, dt, fastSubsteps, st);
            break;
    }
    updateTunnelFlags(w);
}
