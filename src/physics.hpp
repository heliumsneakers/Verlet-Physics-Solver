#pragma once
#include "raylib.h"

enum SimulationMode {
    VERLET,
    FLUID
};

struct Particle {
    Vector2 position;
    Vector2 oldPosition;
    Vector2 acceleration;
    Vector2 velocity;
    float radius;
    Color color; // Pointer to the color

    // SPH specific parameters
    Vector2 force;
    float density;
    float nearDensity;
    float pressure;
    float nearPressure;
    

    // Muller surface tension specific parameters
    Vector2 gradient;
    float curvature;
    float colorField;
};

void InitializeParticles(int count, Color particleColor);
void AssignParticlesToGrid();
void ResolveGridCollisions();
void UpdateVerletParticles(float deltaTime);
void DrawParticles(Color* color);
void SpawnParticle(Vector2 position, float radius, Color color);
void ClearParticles();
void PushParticles(Vector2 position, float force);
void PickUpParticles(Vector2 position, float radius, Vector2 force);
void DrawParticleCount();

// SPH specific functions
void UpdateSPHParticles(float deltaTime);
void DoubleDensityRelaxation(float deltaTime);
void ComputeDensityPressure(void);
void ComputeForces(void);

extern SimulationMode currentMode;

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
extern float sigma;
