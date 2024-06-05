#include "rlImGui.h"
#include "imgui.h"
#include "raylib.h"
#include <cstdlib>
#include <stdio.h>

// custom includes here
#include "physics.hpp"

#define WIDTH 1280
#define HEIGHT 720

Color color = RAYWHITE;
Color bgColor = BLACK;

enum MouseAction {
    NONE,
    SPAWN_PARTICLES,
    PUSH_PARTICLES,
    PICK_PARTICLES
};

MouseAction currentAction = NONE;
SimulationMode currentMode = FLUID;
float particleRadius = 5.0f;
float pushForce = 100.0f;
float pickRadius = 100.0f;
Vector2 pickForce = {-150.0f, -100.0f};
bool dragging = false;

// Convert raylib Color to float array for ImGui
void ColorToFloatArray(const Color& color, float* floatArray) {
    floatArray[0] = color.r / 255.0f;
    floatArray[1] = color.g / 255.0f;
    floatArray[2] = color.b / 255.0f;
}

// Convert float array to raylib Color
Color FloatArrayToColor(const float* floatArray) {
    Color color;
    color.r = static_cast<unsigned char>(floatArray[0] * 255);
    color.g = static_cast<unsigned char>(floatArray[1] * 255);
    color.b = static_cast<unsigned char>(floatArray[2] * 255);
    color.a = 255;
    return color;
}

int main() {
    InitWindow(WIDTH, HEIGHT, "Verlet Integration");

    InitializeParticles(3000, color);

    rlImGuiSetup(true); // init raylib imgui with darkmode

    SetTargetFPS(144);

    float bgColorArray[3];
    float colorArray[3];

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();

        if (currentMode == VERLET) {
            UpdateVerletParticles(deltaTime);
        } else if (currentMode == FLUID) {
            UpdateSPHParticles(deltaTime);
        }

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            Vector2 mousePosition = GetMousePosition();
            if (currentAction == SPAWN_PARTICLES && dragging) {
                SpawnParticle(mousePosition, particleRadius, color);
            } else if (currentAction == PUSH_PARTICLES) {
                PushParticles(mousePosition, -pushForce);
            } else if (currentAction == PICK_PARTICLES) {
                PickUpParticles(mousePosition, pickRadius, pickForce);
            }
            dragging = true;
        } else {
            dragging = false;
        }

        BeginDrawing();

        ClearBackground(bgColor);

        DrawParticles(); // Draw particles first

        rlImGuiBegin(); // start ImGui content mode

        ColorToFloatArray(bgColor, bgColorArray);
        ColorToFloatArray(color, colorArray);

        bool showWindow = ImGui::Begin("Simulation Settings", NULL, ImGuiWindowFlags_AlwaysAutoResize);

        if (showWindow) {
            const char* modes[] = { "Verlet", "Fluid" };
            int currentModeIndex = currentMode;
            ImGui::Combo("Simulation Mode", &currentModeIndex, modes, IM_ARRAYSIZE(modes));
            currentMode = static_cast<SimulationMode>(currentModeIndex);

            const char* actions[] = { "None", "Spawn", "Push", "Pick Up" };
            int currentActionIndex = currentAction;
            ImGui::Combo("Mouse Action", &currentActionIndex, actions, IM_ARRAYSIZE(actions));

            currentAction = static_cast<MouseAction>(currentActionIndex);
            if (currentAction == SPAWN_PARTICLES) {
                ImGui::SliderFloat("Particle Radius", &particleRadius, 2.0f, 10.0f);
            } else if (currentAction == PUSH_PARTICLES) {
                ImGui::SliderFloat("Push Force", &pushForce, 100.0f, 1000.0f);
            }
            

            ImGui::ColorEdit3("Background Color", bgColorArray);
            ImGui::ColorEdit3("Shape Color", colorArray); 
            ImGui::SliderFloat("Gravity", &gravity, -98.1f, 98.1f);

            // Fluid parameters
            if (currentMode == FLUID) {
                ImGui::SliderFloat("Density", &den, 1.0f, 100.0f);
                ImGui::SliderFloat("Near Density", &n_den, 1.0f, 100.0f);
                ImGui::SliderFloat("Pressure", &pres, 1.0f, 100.0f);
                ImGui::SliderFloat("Near Pressure", &n_pres, 1.0f, 100.0f);
                ImGui::SliderFloat("Stiffness", &k, -1.0f, 1.0f);
                ImGui::SliderFloat("Near Pressure Stiffness", &kNear, 0.1f, 1.0f);
                ImGui::SliderFloat("Rest Density", &rho0, 0.1f, 50.0f);
                ImGui::SliderFloat("Smoothing Radius", &h, 0.1f, 32.0f);
            }

            if (ImGui::Button("Clear Particles")) {
                ClearParticles();
            }
        }

        ImGui::End();

        // Convert float arrays back to Colors
        bgColor = FloatArrayToColor(bgColorArray);
        color = FloatArrayToColor(colorArray);

        rlImGuiEnd();

        DrawFPS(WIDTH / 2, 10);

        DrawParticleCount();

        EndDrawing();
    }

    rlImGuiShutdown();

    CloseWindow();

    return 0;
}
