// physics.cpp
#include "physics.hpp"
#include "raymath.h"

std::vector<Particle> particles;

void InitializeParticles(int count, Color* particleColor) {
    for (int i = 0; i < count; ++i) {
        Particle p;
        p.position = { (float)GetRandomValue(100, 700), (float)GetRandomValue(100, 500) };
        p.oldPosition = p.position;
        p.acceleration = { 0, 0 };
        p.radius = 5.0f;
        p.color = particleColor; 
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
        
        p1.position = Vector2Subtract(p1.position, offset);
        p2.position = Vector2Add(p2.position, offset);
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

void UpdateParticles(float deltaTime) {
    for (auto& particle : particles) {
        ApplyForce(particle, { 0, 98.1f }); // Gravity
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

