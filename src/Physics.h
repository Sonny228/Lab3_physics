#pragma once
// Физика: тела-круги, стенки-AABB, три режима шага (Discrete / CCD / Multi-rate).
#include <vector>
#include "Vec2.h"
#include "Config.h"

enum class Mode { Discrete, CCD, MultiRate };
const char* modeName(Mode m);

// Круг. Масса пропорциональна площади: m = MassScale * r^2. invMass = 0 -> неподвижное тело.
struct Body {
    Vec2  pos, vel;
    Vec2  prevStep;          // позиция в начале текущего шага физики (нужна дискретной проверке)
    Vec2  prevFrame;         // позиция в начале "большого" шага (для интерполяции отрисовки)
    float radius  = 1.f;
    float invMass = 0.f;
    bool  useGravity = true;
    bool  hitThin  = false;  // тело уже ударилось о тонкую стенку
    bool  tunneled = false;  // тело оказалось за тонкой стенкой, не ударившись о неё
};

// Стенка - прямоугольник со сторонами вдоль осей (AABB), неподвижная.
struct Wall {
    Vec2 min, max;
    bool thin = false;       // тонкая стенка из задания (для подсчёта туннелирования)
};

// Мир: три группы тел лежат в отдельных списках (для multi-rate).
struct World {
    std::vector<Wall> walls;
    std::vector<Body> statics;  // неподвижные круги
    std::vector<Body> slow;     // обычные тела (60 Гц)
    std::vector<Body> fast;     // быстрые тела (снаряды, 240 Гц)
    float restitution = cfg::Restitution;
    float gravity     = cfg::Gravity;
    int   thinWallIndex = -1;   // индекс тонкой стенки в walls, -1 если её нет

    std::size_t dynamicCount() const { return slow.size() + fast.size(); }
    int tunneledCount() const;  // сколько быстрых тел пролетело сквозь тонкую стенку
};

// Статистика одного большого шага.
struct StepStats {
    int contacts = 0;        // обработано столкновений
    int ccdIterations = 0;   // проходов поиска касаний (CCD)
    int ccdLimitHits = 0;    // сколько раз упёрлись в MaxCcdIterations
    int slowSteps = 0;       // шагов обычных тел
    int fastSteps = 0;       // шагов быстрых тел (подшагов)
};

inline float invMassOfRadius(float r) { return 1.f / (cfg::MassScale * r * r); }

// Один большой шаг dt (шаг обычных тел). В режиме MultiRate быстрые делают k подшагов dt/k.
void stepWorld(World& w, Mode mode, float dt, StepStats& st, int fastSubsteps = cfg::FastSubstepsK);

// Для интерполяции: запомнить позиции перед большим шагом.
void beginMacroStep(World& w);

// Геометрия CCD (вынесены наружу, чтобы использовать в экспериментах).
// Время первого касания двух кругов за время tmax. Возвращает false, если касания нет.
bool sweepCircleCircle(Vec2 p1, Vec2 v1, float r1, Vec2 p2, Vec2 v2, float r2,
                       float tmax, float& tHit);
// Время первого касания круга и стенки (через "раздутый" на r прямоугольник) и нормаль стенки.
bool sweepCircleAABB(Vec2 p, Vec2 v, float r, const Wall& w, float tmax,
                     float& tHit, Vec2& normal);
