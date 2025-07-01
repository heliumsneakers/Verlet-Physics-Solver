#include "physics.hpp"
#include "raylib.h"
#include "raymath.h"
#include <cstddef>
#include <thread>
#include <algorithm>
#include <vector>
#include <cmath>

std::vector<Particle> particles;

// Simulation parameters
float den = 1.0f;              // Density
float n_den = 35.0f;           // Near Density
float pres = 2000.0f;          // Pressure
float n_pres = 1.0f;           // Near Pressure
float k = -0.5f;               // Stiffness parameter
float kNear = 0.5f;            // Near pressure stiffness parameter
float rho0 = 500.0f;           // Rest density
float h = 32.0f;               // Interaction radius || Smoothing radius
float gravity = 98.1f;         
float damping = 0.999f;        // Damping factor for Verlet collisions
float hsq = h * h;
const static float EPS = h;    // Boundary epsilon
const static float BOUND_DAMPING = -1.0f;

// Elasticity and plasticity parameters
float kSpring = 0.3f;          // Spring constant for elasticity
float plasticityAlpha = 0.3f;  // Plasticity rate constant
float yieldRatio = 0.2f;       // Yield ratio for plastic deformation threshold

// Viscosity parameters
float sigma = 0.5f;            // Linear viscosity coefficient
float beta = 0.5f;             // Quadratic viscosity coefficient

float maxVelocity = 1.0f;
float sigmaSurface = 0.0728f;  // Surface tension coefficient for water-air interface

// Parameters for grid traversal
float cellSize = h;
int gridWidth;
int gridHeight;
std::vector<std::vector<std::vector<Particle*>>> grid;

// Structure to hold spring-related data
struct Spring {
    int particleA;
    int particleB;
    float restLength;

    bool operator==(const Spring& other) const {
        return (particleA == other.particleA && particleB == other.particleB && restLength == other.restLength);
    }
};

std::vector<Spring> springs;

// Multithreading helper function
template <typename F>
void ParallelFor(size_t start, size_t end, F&& func) {
    size_t numThreads = std::thread::hardware_concurrency();
    size_t blockSize = (end - start + numThreads - 1) / numThreads;
    std::vector<std::thread> threads(numThreads);

    for (size_t t = 0; t < numThreads; ++t) {
        size_t blockStart = start + t * blockSize;
        size_t blockEnd = std::min(blockStart + blockSize, end);
        threads[t] = std::thread([=, &func]() {
            for (size_t i = blockStart; i < blockEnd; i++) {
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
        p.radius = 5.0f;
        p.color = particleColor;
        p.density = 0.0f;
        p.nearDensity = n_den;
        p.pressure = 0.0f;
        p.nearPressure = n_pres;
        particles.push_back(p);
    }

    gridWidth = (int)ceil(GetScreenWidth() / cellSize);
    gridHeight = (int)ceil(GetScreenHeight() / cellSize);
    grid = std::vector<std::vector<std::vector<Particle*>>>(gridWidth, std::vector<std::vector<Particle*>>(gridHeight));
}

void AdjustSprings() {
    for (auto& spring : springs) {
        Particle& pa = particles[spring.particleA];
        Particle& pb = particles[spring.particleB];

        float currentLength = Vector2Length(Vector2Subtract(pa.position, pb.position));
        float deltaLength = currentLength - spring.restLength;

        // Update spring rest length based on plastic deformation
        if (std::fabs(deltaLength) > yieldRatio * spring.restLength) {
            spring.restLength += plasticityAlpha * deltaLength;
        }

        if (spring.restLength > h) {
            springs.erase(std::remove(springs.begin(), springs.end(), spring), springs.end());
        }
    }
}

void ApplySpringDisplacements(float deltaTime) {
    for (const auto& spring : springs) {
        Particle& pa = particles[spring.particleA];
        Particle& pb = particles[spring.particleB];

        float currentLength = Vector2Length(Vector2Subtract(pa.position, pb.position));
        Vector2 direction = Vector2Normalize(Vector2Subtract(pb.position, pa.position));
        float displacementMagnitude = deltaTime * deltaTime * kSpring * (1.0f - spring.restLength / h) * (spring.restLength - currentLength);

        Vector2 displacement = Vector2Scale(direction, displacementMagnitude);
        pa.position = Vector2Subtract(pa.position, Vector2Scale(displacement, 0.5f));
        pb.position = Vector2Add(pb.position, Vector2Scale(displacement, 0.5f));
    }
}

void ApplyViscosity(float deltaTime) {
    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            Particle& pi = particles[i];
            Particle& pj = particles[j];

            float distance = Vector2Length(Vector2Subtract(pi.position, pj.position));
            if (distance < h) {
                Vector2 direction = Vector2Normalize(Vector2Subtract(pj.position, pi.position));
                float radialVelocity = Vector2DotProduct(Vector2Subtract(pj.velocity, pi.velocity), direction);

                if (radialVelocity > 0) {
                    float q = distance / h;
                    float impulseMagnitude = deltaTime * (1 - q) * (sigma * radialVelocity + beta * radialVelocity * radialVelocity);

                    Vector2 impulse = Vector2Scale(direction, impulseMagnitude);
                    pi.velocity = Vector2Subtract(pi.velocity, Vector2Scale(impulse, 0.5f));
                    pj.velocity = Vector2Add(pj.velocity, Vector2Scale(impulse, 0.5f));
                }
            }
        }
    }
}

void AssignParticlesToGrid() {
    for (auto& row : grid) {
        for (auto& cell : row) {
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

void ApplyForce(Particle& p, Vector2 force) {
    p.acceleration = Vector2Add(p.acceleration, force);
}

Color VelocityToColor(const Vector2& velocity, float maxVelocity) {
    float speed = Vector2Length(velocity);
    float t = speed / maxVelocity * 2.0f;
    unsigned char r = static_cast<unsigned char>(0 * (1 - t) + 173 * t);
    unsigned char g = static_cast<unsigned char>(0 * (1 - t) + 216 * t);
    unsigned char b = static_cast<unsigned char>(139 * (1 - t) + 230 * t);
    return (Color){ r, g, b, 255 };
}

void VerletIntegration(Particle& p, float deltaTime) {
    Vector2 temp = p.position;
    p.velocity = Vector2Subtract(p.position, p.oldPosition);
    // First Iteration: removed 1/2 accel -- particles had greater instability
    // p.position = Vector2Add(p.position, p.velocity);
    // p.position = Vector2Add(p.position, Vector2Scale(p.acceleration, deltaTime * deltaTime));

    // Second Iteration: Direct calculation of the Verlet Integration formula including 1/2accel for stability
    p.position = Vector2Add(p.position, Vector2Add(p.velocity, Vector2Scale(Vector2Scale(p.acceleration, 0.5f), (deltaTime * deltaTime))));
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

void FluidBounds(Particle& p, int screenWidth, int screenHeight) {
    float minX = p.radius;
    float maxX = screenWidth - p.radius;
    float minY = p.radius;
    float maxY = screenHeight - p.radius;
    
    // Constrain X-axis
    if (p.position.x < minX) {
        p.position.x = minX;
        p.velocity.x *= BOUND_DAMPING; // Reverse and dampen velocity
    } else if (p.position.x > maxX) {
        p.position.x = maxX;
        p.velocity.x *= BOUND_DAMPING;
    }

    // Constrain Y-axis
    if (p.position.y < minY) {
        p.position.y = minY;
        p.velocity.y *= BOUND_DAMPING;
    } else if (p.position.y > maxY) {
        p.position.y = maxY;
        p.velocity.y *= BOUND_DAMPING;
    }
}

 void DoubleDensityRelaxation(float deltaTime) {
    ParallelFor(0, particles.size(), [&](size_t i) {
        Particle& pi = particles[i];
        pi.density = 0.0f;
        pi.nearDensity = 0.0f;

        // Neighbor cache for efficiency
        std::vector<int> neighbors;
        std::vector<float> neighborCloseness;
        std::vector<Vector2> neighborDirection;

        // Compute density and near-density with neighbors
        for (size_t j = 0; j < particles.size(); ++j) {
            if (i == j) continue;

            Particle& pj = particles[j];
            Vector2 rij = Vector2Subtract(pj.position, pi.position);
            float r = Vector2Length(rij);

            if (r < h) {
                float q = r / h;
                float closeness = 1 - q;
                float closenessSq = closeness * closeness;

                pi.density += closeness * closeness;
                pi.nearDensity += closeness * closenessSq;

                neighbors.push_back(j);
                neighborCloseness.push_back(closeness);
                neighborDirection.push_back(Vector2Normalize(rij));
            }
        }

        // Compute pressure and near-pressure
        float pressure = k * (pi.density - rho0);
        float nearPressure = kNear * pi.nearDensity;

        Vector2 displacement = {0, 0};

        // Apply displacements to neighbors based on pressure and near-pressure
        for (size_t n = 0; n < neighbors.size(); ++n) {
            int neighborIdx = neighbors[n];
            Particle& pj = particles[neighborIdx];

            float closeness = neighborCloseness[n];
            Vector2 direction = neighborDirection[n];

            Vector2 displacementContribution = Vector2Scale(direction,
                (deltaTime * deltaTime) *
                (pressure * closeness + nearPressure * closeness * closeness));

            // Apply action-reaction
            pj.position = Vector2Add(pj.position, Vector2Scale(displacementContribution, 0.5f));
            displacement = Vector2Subtract(displacement, Vector2Scale(displacementContribution, 0.5f));
        }

        // Update the particle's position
        pi.position = Vector2Add(pi.position, displacement);
    });
}

void UpdateSPHParticles(float deltaTime) {
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    ApplyViscosity(deltaTime);
    ParallelFor(0, particles.size(), [&](size_t i) {
        ApplyForce(particles[i], { 0, gravity });
        VerletIntegration(particles[i], deltaTime);

        float speed = Vector2Length(particles[i].velocity);
        if (speed > maxVelocity) {
            maxVelocity = speed;
        }
        particles[i].color = VelocityToColor(particles[i].velocity, maxVelocity);
        FluidBounds(particles[i], screenWidth, screenHeight);
    });

    AdjustSprings();
    ApplySpringDisplacements(deltaTime);
    DoubleDensityRelaxation(deltaTime);
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
