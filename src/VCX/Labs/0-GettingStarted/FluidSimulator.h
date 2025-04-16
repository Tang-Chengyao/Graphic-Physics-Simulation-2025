#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::Fluid {
    struct Simulator {
        std::vector<glm::vec3> Positions;
        std::vector<glm::vec3> InitPositions;
        std::vector<glm::vec3> Velocities;

        float Omega { 0.5f };
        float Mass { 1.f };
        float Damping { .1f };

        void SimulateTimestep(float const dt) {
            for (std::size_t i = 0; i < Positions.size(); i++) {
                glm::vec3 acceleration = glm::vec3(-(Positions[i].y + 1.0) * pow(Omega, 2) * (Positions[i].x - InitPositions[i].x), 0.0, 0.0) / Mass
                    - Damping * Velocities[i] / Mass
                    + glm::vec3(0.0f, -9.81f, 0.0f);
                Velocities[i] += acceleration * dt;
                Positions[i] += Velocities[i] * dt;
            }
        }
    };
}