#include "physics.hpp"
#include "raymath.h"
#include <vector>

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
float damping   = 0.999f;     // Damping factor for Verlet collisions

float maxVelocity = 1.0f;

void InitializeParticles(int count, Color particleColor) {
    for (int i = 0; i < count; ++i) {
        Particle p;
        p.position = { (float)GetRandomValue(100, 700), (float)GetRandomValue(100, 500) };
        p.oldPosition = p.position;
        p.acceleration = { 0, 0 };
        p.velocity = { 0, 0 };
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

float GetParticleVelocity(const Particle& p) {
    Vector2 velocity = Vector2Subtract(p.position, p.oldPosition);
    return Vector2Length(velocity);
}

Color VelocityToColor(const Vector2& velocity, float maxVelocity) {
    float speed = Vector2Length(velocity);
    float t = speed / maxVelocity;
    unsigned char r = static_cast<unsigned char>(0 * (1 - t) + 173 * t);
    unsigned char g = static_cast<unsigned char>(0 * (1 - t) + 216 * t);
    unsigned char b = static_cast<unsigned char>(139 * (1 - t) + 230 * t);
    return (Color){ r, g, b, 255 };
}

void VerletIntegration(Particle& p, float deltaTime) {
    Vector2 temp = p.position;
    p.velocity = Vector2Subtract(p.position, p.oldPosition);  // Calculate velocity
    p.position = Vector2Add(p.position, p.velocity);
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
        particle.color = VelocityToColor(particle.velocity, maxVelocity);

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

    DoubleDensityRelaxation(deltaTime);

    for (auto& particle : particles) {
        ApplyForce(particle, { 0, gravity }); // Gravity
        VerletIntegration(particle, deltaTime);

        // Update particle color based on velocity
        float speed = Vector2Length(particle.velocity);
        if (speed > maxVelocity) {
            maxVelocity = speed;
        }
        particle.color = VelocityToColor(particle.velocity, maxVelocity);
    }
 
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();
    for (auto& particle : particles) {
        ConstrainToBounds(particle, screenWidth, screenHeight); 
    }
}

void DoubleDensityRelaxation(float deltaTime) {
    // Calculate density
    for (auto& particle : particles) {
        particle.density = den;
        particle.nearDensity = n_den;
        for (const auto& neighbor : particles) {
            Vector2 delta = Vector2Subtract(neighbor.position, particle.position);
            float r = Vector2Length(delta);
            if (r < h){
                float q = r/h;
                particle.density += (1 - q) * (1 - q);
                particle.nearDensity += (1 - q) * (1 - q) * (1 - q);
            }
        }
        // Calculate pressures
        particle.pressure = k * (particle.density - rho0);
        particle.nearPressure = kNear * particle.nearDensity;
        Vector2 dx = { 0, 0 };

        // Apply displacements
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

void DrawParticles(Color* color) {
    for (const auto& particle : particles) {
        if (currentMode == FLUID){
        DrawCircleV(particle.position, particle.radius, particle.color);
        } else if (currentMode == VERLET){
        DrawCircleV(particle.position, particle.radius, *color);
        } 
    }
}

void SpawnParticle(Vector2 position, float radius, Color color) {
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

void PickUpParticles(Vector2 position, float radius, Vector2 force) {
    for (auto& particle : particles) {
        Vector2 delta = Vector2Subtract(particle.position, position);
        float distance = Vector2Length(delta);
        if (distance < radius) {
            Vector2 normalized = Vector2Scale(delta, 1.0f / distance);
            ApplyForce(particle, Vector2Scale(normalized, force.x));
            ApplyForce(particle, {0, force.y}); // Separate the x and y components of the force
        }
    }
}

void DrawParticleCount() {
    char countText[50];
    snprintf(countText, sizeof(countText), "Particles: %zu", particles.size());
    int textWidth = MeasureText(countText, 20);
    DrawText(countText, GetScreenWidth() - textWidth - 10, 10, 20, DARKGRAY); // Position the text in the top right corner
}
