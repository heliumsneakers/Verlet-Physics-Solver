#include "physics.hpp"
#include "raylib.h"
#include "raymath.h"

#include <cstddef>
#include <thread>
#include <algorithm>
#include <vector>

std::vector<Particle> particles;

float den           = 1.0f;      // Density
float n_den         = 1000.0f;   // Near Density
float pres          = 1000.0f;   // Pressure
float n_pres        = 1.0f;      // Near Pressure
float k             = -0.01f;    // Stiffness parameter
float kNear         = 1.5f;      // Near pressure stiffness parameter
float rho0          = 50.0f;     // Rest density
float h             = 8.0f;      // Interaction radius || Smoothing radius
float gravity       = 98.1f;     
float damping       = 0.999f;    // Damping factor for Verlet collisions

float maxVelocity   = 1.0f;
float sigma         = 0.0728f;   // Surface tension coefficient for water-air interface
         

// Parameters for grid traversal
float cellSize = h;
int gridWidth;
int gridHeight;
std::vector<std::vector<std::vector<Particle*>>> grid;

// Helper function to split workloads. Multithreading for the update functions and physics calculations.
template <typename F>
void ParallelFor(size_t start, size_t end, F&& func){
    size_t numThreads = std::thread::hardware_concurrency();
    size_t blockSize = (end - start + numThreads - 1) / numThreads; // Ceiling division.
    std::vector<std::thread> threads(numThreads);
    
    for (size_t t = 0; t < numThreads; ++t){
        size_t blockStart = start + t * blockSize;
        size_t blockEnd = std::min(blockStart + blockSize, end);
        threads[t] = std::thread([=, &func]() {
            for (size_t i = blockStart; i < blockEnd; i++){
                func(i);
            }    
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }
}

void InitializeParticles(int count, Color particleColor) {
    for (int i = 0; i < count; ++i) {
        Particle p;
        p.position = { (float)GetRandomValue(100, 700), (float)GetRandomValue(100, 500) };
        p.oldPosition = p.position;
        p.acceleration = { 0, 0 };
        p.velocity = { 0, 0 };
        p.radius = 8.0f;
        p.color = particleColor;
        p.density = den;
        p.nearDensity = n_den;
        p.pressure = pres;
        p.nearPressure = n_pres;
        particles.push_back(p);
    }

    // Init grid with width and height based on particle smoothing radius and current screen size.
    gridWidth = (int)ceil(GetScreenWidth() / cellSize);
    gridHeight = (int)ceil(GetScreenHeight() / cellSize);
    grid = std::vector<std::vector<std::vector<Particle*>>>(gridWidth, std::vector<std::vector<Particle*>>(gridHeight));
}

void AssignParticlesToGrid() {
    for (auto& row : grid) {
        for (auto& cell : row){
            cell.clear();
        }
    }

    for (auto& particle : particles) {
        int cellX = (int)(particle.position.x / cellSize);
        int cellY = (int)(particle.position.y / cellSize);
        if (cellX >= 0 && cellX < gridWidth && cellY >= 0 && cellY < gridHeight) {
            grid[cellX][cellY].push_back(&particle);
        }
    }
}

float KernelPoly6(float r, float h) {
    float result = 0.0f;
    if (r >= 0 && r <= h) {
        float x = (h * h) - (r * r);
        result = (315.0f / (64.0f * PI * pow(h, 9))) * pow(x, 3);
    }
    return result;
}

Vector2 KernelPoly6Grad(Vector2 r, float h) {
    Vector2 result = {0, 0};
    float r_len = Vector2Length(r);
    if (r_len >= 0 && r_len <= h) {
        float x = (h * h) - (r_len * r_len);
        result = Vector2Scale(r, (-945.0f / (32.0f * PI * pow(h, 9))) * x * x);
    }
    return result;
}

float KernelPoly6Laplacian(float r, float h) {
    float result = 0.0f;
    if (r >= 0 && r <= h) {
        float x = (h * h) - (r * r);
        result = (945.0f / (32.0f * PI * pow(h, 9))) * (x * (3 * (h * h) - 7 * (r * r)));
    }
    return result;
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

void ResolveGridCollisions() {
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& p1 = particles[i];
        int cellX = (int)(p1.position.x / cellSize);
        int cellY = (int)(p1.position.y / cellSize);

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* p2 : grid[x][y]) {
                    if (&p1 != p2) {
                        ResolveCollision(p1, *p2);
                    } }
            }
        }
    });
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

void CalcColorFGrad() {
    // Calculate the color field c_S
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& particle = particles[i];
        particle.colorField = 0.0f;

        int cellX = (int)(particle.position.x / cellSize);
        int cellY = (int)(particle.position.y / cellSize);

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* neighbor : grid[x][y]) {
                    Vector2 delta = Vector2Subtract(neighbor->position, particle.position);
                    float r = Vector2Length(delta);
                    if (r < h) {
                        float W = KernelPoly6(r, h);
                        particle.colorField += (1.0f / neighbor->density) * W;
                    }
                }
            }
        }
    });

    // Calculate the gradient of the color field
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& particle = particles[i];
        particle.gradient = {0, 0};

        int cellX = (int)(particle.position.x / cellSize);
        int cellY = (int)(particle.position.y / cellSize);

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* neighbor : grid[x][y]) {
                    Vector2 delta = Vector2Subtract(neighbor->position, particle.position);
                    float r = Vector2Length(delta);
                    if (r < h) {
                        float W = KernelPoly6(r, h);
                        Vector2 gradW = KernelPoly6Grad(delta, h);
                        particle.gradient = Vector2Add(particle.gradient, Vector2Scale(gradW, 1.0f / neighbor->density));
                    }
                }
            }
        }
    });
}

void CalculateCurvature() {
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& particle = particles[i];
        particle.curvature = 0.0f;

        int cellX = (int)(particle.position.x / cellSize);
        int cellY = (int)(particle.position.y / cellSize);

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* neighbor : grid[x][y]) {
                    Vector2 delta = Vector2Subtract(neighbor->position, particle.position);
                    float r = Vector2Length(delta);
                    if (r < h) {
                        float W = KernelPoly6(r, h);
                        float laplacianW = KernelPoly6Laplacian(r, h);
                        particle.curvature += (1.0f / neighbor->density) * laplacianW;
                    }
                }
            }
        }

        particle.curvature = -particle.curvature / Vector2Length(particle.gradient);
    });
}

void ApplySurfaceTension(float deltaTime) {
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& particle = particles[i];
        if (Vector2Length(particle.gradient) > 1e-5f) {  // Threshold to avoid division by zero
            Vector2 normal = Vector2Scale(particle.gradient, 1.0f / Vector2Length(particle.gradient));
            Vector2 surfaceTensionForce = Vector2Scale(normal, -sigma * particle.curvature);
            ApplyForce(particle, surfaceTensionForce);
        }
    });
}

void UpdateVerletParticles(float deltaTime) {
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    ParallelFor(0, particles.size(), [&](size_t i) {
        ApplyForce(particles[i], { 0, gravity });
        VerletIntegration(particles[i], deltaTime);
        ConstrainToBounds(particles[i], screenWidth, screenHeight);
    });

    AssignParticlesToGrid();
    ResolveGridCollisions();
}

void UpdateSPHParticles(float deltaTime) {
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    ParallelFor(0, particles.size(), [&](size_t i) {
        ApplyForce(particles[i], { 0, gravity });
        VerletIntegration(particles[i], deltaTime);

        float speed = Vector2Length(particles[i].velocity);
        if (speed > maxVelocity) {
            maxVelocity = speed;
        }
        particles[i].color = VelocityToColor(particles[i].velocity, maxVelocity);
        ConstrainToBounds(particles[i], screenWidth, screenHeight);
    });

    AssignParticlesToGrid();
    DoubleDensityRelaxation(deltaTime);
    CalcColorFGrad();
    CalculateCurvature();
    ApplySurfaceTension(deltaTime);
}

void DoubleDensityRelaxation(float deltaTime) {
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& particle = particles[i];
        particle.density = den;
        particle.nearDensity = n_den;

        int cellX = (int)(particle.position.x / cellSize);
        int cellY = (int)(particle.position.y / cellSize);

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* neighbor : grid[x][y]) {
                    Vector2 delta = Vector2Subtract(neighbor->position, particle.position);
                    float r = Vector2Length(delta);
                    if (r < h) {
                        float q = r / h;
                        particle.density += (1 - q) * (1 - q);
                        particle.nearDensity += (1 - q) * (1 - q) * (1 - q);
                    }
                }
            }
        }

        particle.pressure = k * (particle.density - rho0);
        particle.nearPressure = kNear * particle.nearDensity;
        Vector2 dx = { 0, 0 };

        for (int x = std::max(0, cellX - 1); x <= std::min(gridWidth - 1, cellX + 1); ++x) {
            for (int y = std::max(0, cellY - 1); y <= std::min(gridHeight - 1, cellY + 1); ++y) {
                for (Particle* neighbor : grid[x][y]) {
                    Vector2 delta = Vector2Subtract(neighbor->position, particle.position);
                    float r = Vector2Length(delta);
                    if (r < h) {
                        float q = r / h;
                        Vector2 D = Vector2Scale(delta, (deltaTime * deltaTime) * (particle.pressure * (1 - q) + particle.nearPressure * (1 - q) * (1 - q)));
                        neighbor->position = Vector2Add(neighbor->position, Vector2Scale(D, 0.5f));
                        dx = Vector2Subtract(dx, Vector2Scale(D, 0.5f));
                    }
                }
            }
        }

        particle.position = Vector2Add(particle.position, dx);
    });
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
