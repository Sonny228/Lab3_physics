#pragma once
// =====================================================================
//  Config.h - ВСЕ числовые параметры работы в одном месте.
//  Чтобы подставить свои числа, меняется только этот файл.
//  Значения по умолчанию взяты из отчёта коллеги (J3203, Пак А.А.).
// =====================================================================
#include <cstddef>
#include <string>
#include <vector>

namespace cfg {

// ---------------- окно и отрисовка ----------------
inline constexpr unsigned WindowW = 1280;
inline constexpr unsigned WindowH = 720;
inline constexpr bool     VSync   = true;
inline constexpr float    HudPeriod = 0.5f;        // усреднение HUD, с
inline constexpr float    WallThickness = 100.f;   // толщина стен комнаты (стены снаружи окна)

// ---------------- общая физика ----------------
inline constexpr float  Gravity          = 980.f;  // px/с^2 (1 px ~ 1 см)
inline constexpr float  Restitution      = 0.8f;   // e
inline constexpr float  MassScale        = 1.f;    // масса m = MassScale * r^2
inline constexpr int    DefaultHz        = 60;     // частота физики по умолчанию, Гц
inline constexpr int    FreqPresets[3]   = {60, 120, 240};  // клавиши 4, 5, 6
inline constexpr int    FastSubstepsK    = 4;      // подшагов быстрых на 1 шаг обычных (60 -> 240 Гц)
inline constexpr int    MaxCcdIterations = 100;    // лимит касаний CCD за шаг
inline constexpr double MaxFrameTime     = 0.25;   // обрезка кадра в аккумуляторе, с

// ---------------- задание: снаряд и тонкая стенка ----------------
inline constexpr float ProjectileSpeed   = 2000.f; // px/с
inline constexpr float ProjectileRadius  = 5.f;    // px
inline constexpr float ThinWallThickness = 10.f;   // px
inline constexpr float ThinWallX0        = 640.f;  // ближняя грань стенки (слева), px
inline constexpr int   TunnelProjectiles = 10;     // число снарядов в сцене Tunnel
inline constexpr float TunnelX0          = 100.f;  // x первого снаряда
inline constexpr float TunnelY0          = 110.f;  // y первого снаряда
inline constexpr float TunnelYStep       = 55.f;   // шаг по y между снарядами
inline constexpr float TunnelPhaseDt     = 1.f / 60.f; // dt в формуле x_i = X0 + i*v*dt/N

// ---------------- сцена Room (сцена 1) ----------------
inline constexpr int   RoomBalls      = 5;
inline constexpr float RoomRadiusMin  = 15.f;
inline constexpr float RoomRadiusStep = 5.f;       // r_i = Min + i*Step  (15..35)

// ---------------- сцена Multi-rate (сцена 3) ----------------
inline constexpr unsigned Seed          = 12345;   // seed генератора (единственная случайность)
inline constexpr int   StaticCount      = 6;       // ТЗ: 3..10
inline constexpr int   SlowCount        = 40;      // ТЗ: 10..100
inline constexpr int   FastCount        = 5;       // ТЗ: 1..10
inline constexpr float SlowRadiusMin    = 12.f;
inline constexpr float SlowRadiusMax    = 20.f;
inline constexpr float StaticRadius     = 2.f * SlowRadiusMax;  // ТЗ: размер x2 от обычных (=40)
inline constexpr float StaticMarginX    = 120.f;
inline constexpr float StaticY0         = 400.f;
inline constexpr float StaticYAlt       = 80.f;    // чётные/нечётные статики сдвинуты по y
inline constexpr float SlowAreaYMin     = 40.f;    // область появления обычных шаров
inline constexpr float SlowAreaYMax     = 300.f;
inline constexpr float SlowSpeedMaxX    = 150.f;
inline constexpr float SlowSpeedMaxY    = 50.f;
inline constexpr float FastX0           = 60.f;
inline constexpr float FastY0           = 80.f;
inline constexpr float FastYStep        = 130.f;
inline constexpr float FastVyStep       = 60.f;    // vy_i = (i - FastCount/2) * FastVyStep

// ---------------- сцена Stack (сцена S) ----------------
inline constexpr int   StackRows   = 8;            // 8+7+...+1 = 36 шаров
inline constexpr float StackRadius = 20.f;
inline constexpr float StackGap    = 0.05f;         // зазор между шарами при старте

// ---------------- эксперименты (клавиша T / pz3_experiments) ----------------
inline constexpr int    TunnelPhases      = 100;
inline constexpr float  TunnelSpeeds[]    = {1000.f, 2000.f, 3000.f, 4000.f, 5000.f, 6000.f, 8000.f};
inline constexpr double DeterminismSecs   = 10.0;
inline constexpr double FloatDtMin        = 1.0 / 70.0;
inline constexpr double FloatDtMax        = 1.0 / 50.0;
inline constexpr unsigned FloatDtSeedA    = 1001;
inline constexpr unsigned FloatDtSeedB    = 2002;
inline constexpr double TimingSecs        = 10.0;
inline constexpr double ReferenceFps      = 200.0;   // FPS из HUD, для пересчёта "мс за кадр"
inline constexpr double StackSecs         = 5.0;
inline constexpr double StackMeasureSecs  = 1.0;     // последняя секунда стека
inline constexpr float  StackStandSink    = 10.f;    // "стоит": проседание < px ...
inline constexpr float  StackStandSpeed   = 20.f;    // ... и средняя |v| < px/с

// ---------------- числа для примеров расчёта в отчёте (п. 1.2) ----------------
inline constexpr float ExampleThrowSpeed = 200.f;   // бросок вверх, px/с
inline constexpr float ExampleY0         = 200.f;   // начальная y, px
inline constexpr float ExampleCircleDist = 100.f;   // расстояние до шара в примере с квадратным уравнением, px

// ---------------- проверка параметров на соответствие ТЗ ----------------
inline std::vector<std::string> validate() {
    std::vector<std::string> w;
    if (StaticCount < 3 || StaticCount > 10) w.push_back("TZ: StaticCount must be 3..10");
    if (SlowCount < 10 || SlowCount > 100)   w.push_back("TZ: SlowCount must be 10..100");
    if (FastCount < 1 || FastCount > 10)     w.push_back("TZ: FastCount must be 1..10");
    if (TunnelProjectiles < 1 || TunnelProjectiles > 10) w.push_back("TZ: fast objects (TunnelProjectiles) must be 1..10");
    if (StaticRadius < 2.f * SlowRadiusMax)  w.push_back("TZ: static size must be x2 of regular objects (StaticRadius >= 2*SlowRadiusMax)");
    if (ProjectileSpeed != 2000.f)           w.push_back("TZ demo: projectile speed is 2000 px/s");
    if (ThinWallThickness != 10.f)           w.push_back("TZ demo: wall thickness is 10 px");
    if (DefaultHz != 60)                     w.push_back("TZ demo: base physics rate is 60 Hz");
    if (FastSubstepsK != 4)                  w.push_back("TZ: fast objects at 240 Hz = 4 substeps per 60 Hz step");
    return w;
}

} // namespace cfg
