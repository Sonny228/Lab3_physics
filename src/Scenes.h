#pragma once
// Сцены: Room (1), Tunnel (2), Multi-rate (3), Stack (S).
#include "Physics.h"

enum class SceneId { Room, Tunnel, MultiRate, Stack };
const char* sceneName(SceneId id);

// Параметры, которые можно менять в рантайме (стрелки), остальное берётся из Config.h
struct SceneParams {
    float projectileSpeed = cfg::ProjectileSpeed;
    float restitution     = cfg::Restitution;
};

void buildScene(World& w, SceneId id, const SceneParams& p = SceneParams());
