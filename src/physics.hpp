#pragma once
#include "raylib.h"
#include <vector>

struct Particle {
    Vector2 position;
    Vector2 oldPosition;
    Vector2 acceleration;
    float radius;
    Color* color; // Pointer to the color
};

void InitializeParticles(int count, Color* particleColor);
void UpdateParticles(float deltaTime);
void DrawParticles();
void SpawnParticle(Vector2 position, float radius, Color* color);
void ClearParticles();
void PushParticles(Vector2 position, float force);
void DrawParticleCount();


