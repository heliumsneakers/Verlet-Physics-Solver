#pragma once
#include "raylib.h"
#include <vector>

struct Particle {
    Vector2 position;
    Vector2 oldPosition;
    Vector2 acceleration;
    float radius;
    Color* color; // Pointer to the color

    // SPH specific properties
    float density;
    float nearDensity;
    float pressure;
    float nearPressure;
};

void InitializeParticles(int count, Color* particleColor);
void UpdateVerletParticles(float deltaTime);
void DrawParticles();
void SpawnParticle(Vector2 position, float radius, Color* color);
void ClearParticles();
void PushParticles(Vector2 position, float force);
void PickUpParticles(Vector2 position, float radius, Vector2 force);
void DrawParticleCount();

// SPH specific functions
void UpdateSPHParticles(float deltaTime);
void ComputeDensities();
void ComputePressures();
void ComputeDisplacements(float deltaTime);

extern float den;
extern float n_den;
extern float pres;
extern float n_pres;
extern float k;
extern float kNear;
extern float rho0;
extern float h;
extern float gravity;
extern float damping;
