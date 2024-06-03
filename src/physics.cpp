#include "physics.hpp"
#include "raymath.h"

std::vector<Particle> particles;

float den       = 1.0f;      // Density
float n_den     = 1000.0f;   // Near Density
float pres      = 1000.0f;   // Pressure
float n_pres    = 1.0f;      // Near Pressure
float k         = -0.01f;    // Stiffness parameter
float kNear     = 1.5f;      // Near pressure stiffness parameter
float rho0      = 3000.0f;   // Rest density
float h         = 8.0f;      // Interaction radius || Smoothing radius
float gravity   = 98.1f;     
float damping   = 0.98f;     // Damping factor for Verlet collisions



void InitializeParticles(int count, Color* particleColor) {
    for (int i = 0; i < count; ++i) {
        Particle p;
        p.position = { (float)GetRandomValue(100, 700), (float)GetRandomValue(100, 500) };
        p.oldPosition = p.position;
        p.acceleration = { 0, 0 };
        p.radius = 5.0f;
        p.color = particleColor;
        p.density = den;
        p.nearDensity = n_den;
        p.pressure = pres;
        p.nearPressure = n_pres;
        particles.push_back(p);
    }
}

void ApplyForce(Particle& p, Vector2 force) {
    p.acceleration = Vector2Add(p.acceleration, force);
}

void VerletIntegration(Particle& p, float deltaTime) {
    Vector2 temp = p.position;
    p.position = Vector2Add(p.position, Vector2Subtract(p.position, p.oldPosition));
    p.position = Vector2Add(p.position, Vector2Scale(p.acceleration, deltaTime * deltaTime));
    p.oldPosition = temp;
    p.acceleration = { 0, 0 };
}

void ResolveCollision(Particle& p1, Particle& p2) {
    Vector2 delta = Vector2Subtract(p1.position, p2.position);
    float distance = Vector2Length(delta);
    float minDistance = p1.radius + p2.radius;

    if (distance < minDistance) {
        float overlap = 0.1f * (distance - minDistance);
        Vector2 offset = Vector2Scale(delta, overlap / distance);

        // Adjust positions to resolve collision
        p1.position = Vector2Subtract(p1.position, offset);
        p2.position = Vector2Add(p2.position, offset);

        // Apply damping to the velocities
        Vector2 p1Velocity = Vector2Subtract(p1.position, p1.oldPosition);
        Vector2 p2Velocity = Vector2Subtract(p2.position, p2.oldPosition);

        p1Velocity = Vector2Scale(p1Velocity, damping);
        p2Velocity = Vector2Scale(p2Velocity, damping);

        p1.oldPosition = Vector2Subtract(p1.position, p1Velocity);
        p2.oldPosition = Vector2Subtract(p2.position, p2Velocity);
    }
}

void ConstrainToBounds(Particle& p, int screenWidth, int screenHeight) {
    if (p.position.x < p.radius) {
        p.position.x = p.radius;
    } else if (p.position.x > screenWidth - p.radius) {
        p.position.x = screenWidth - p.radius;
    }

    if (p.position.y < p.radius) {
        p.position.y = p.radius;
    } else if (p.position.y > screenHeight - p.radius) {
        p.position.y = screenHeight - p.radius;
    }
}

void UpdateVerletParticles(float deltaTime) {
    for (auto& particle : particles) {
        ApplyForce(particle, { 0, gravity }); // Gravity
        VerletIntegration(particle, deltaTime);
    }

    // Resolve collisions
    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            ResolveCollision(particles[i], particles[j]);
        }
    }

    // Constrain particles to screen bounds
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();
    for (auto& particle : particles) {
        ConstrainToBounds(particle, screenWidth, screenHeight);
    }
}

void UpdateSPHParticles(float deltaTime) {
    for (auto& particle : particles) {
        ApplyForce(particle, { 0, gravity }); // Gravity
    }
    ComputeDensities();
    ComputePressures();
    ComputeDisplacements(deltaTime);

    for (auto& particle : particles) {
        VerletIntegration(particle, deltaTime);
    }

    // Constrain particles to screen bounds
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();
    for (auto& particle : particles) {
        ConstrainToBounds(particle, screenWidth, screenHeight);
    }
}

void ComputeDensities() {
    for (auto& particle : particles) {
        particle.density = den;
        particle.nearDensity = n_den;
        for (const auto& neighbor : particles) {
            Vector2 delta = Vector2Subtract(neighbor.position, particle.position);
            float r = Vector2Length(delta);
            if (r < h) {
                float q = r / h;
                particle.density += (1 - q) * (1 - q);
                particle.nearDensity += (1 - q) * (1 - q) * (1 - q);
            }
        }
    }
}

void ComputePressures() {
    for (auto& particle : particles) {
        particle.pressure = k * (particle.density - rho0);
        particle.nearPressure = kNear * particle.nearDensity;
    }
}

void ComputeDisplacements(float deltaTime) {
    for (auto& particle : particles) {
        Vector2 dx = { 0, 0 };
        for (auto& neighbor : particles) {
            Vector2 delta = Vector2Subtract(neighbor.position, particle.position);
            float r = Vector2Length(delta);
            if (r < h) {
                float q = r / h;
                Vector2 D = Vector2Scale(delta, (deltaTime * deltaTime) * (particle.pressure * (1 - q) + particle.nearPressure * (1 - q) * (1 - q)));
                neighbor.position = Vector2Add(neighbor.position, Vector2Scale(D, 0.5f));
                dx = Vector2Subtract(dx, Vector2Scale(D, 0.5f));
            }
        }
        particle.position = Vector2Add(particle.position, dx);
    }
}

void DrawParticles() {
    for (const auto& particle : particles) {
        DrawCircleV(particle.position, particle.radius, *(particle.color));
    }
}

void SpawnParticle(Vector2 position, float radius, Color* color) {
    Particle p;
    p.position = position;
    p.oldPosition = position;
    p.acceleration = { 0, 0 };
    p.radius = radius;
    p.color = color;
    p.density = den;
    p.nearDensity = n_den;
    p.pressure = pres;
    p.nearPressure = n_pres;
    particles.push_back(p);
}

void ClearParticles() {
    particles.clear();
}

void PushParticles(Vector2 position, float force) {
    for (auto& particle : particles) {
        Vector2 direction = Vector2Subtract(particle.position, position);
        float distance = Vector2Length(direction);
        if (distance < 1.0f) distance = 1.0f; // Avoid division by zero
        Vector2 normalized = Vector2Scale(direction, 1.0f / distance);
        ApplyForce(particle, Vector2Scale(normalized, force));
    }
}

void DrawParticleCount() {
    char countText[50];
    snprintf(countText, sizeof(countText), "Particles: %zu", particles.size());
    int textWidth = MeasureText(countText, 20);
    DrawText(countText, GetScreenWidth() - textWidth - 10, 10, 20, DARKGRAY); // Position the text in the top right corner
}
