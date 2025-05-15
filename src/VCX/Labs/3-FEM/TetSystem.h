#pragma once

#include "Labs/3-FEM/Tetrahedron.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct TetSystem {
        float     Lx { 8 };
        float     Ly { 2 };
        float     Lz { 2 }; // length, width, height
        glm::vec4 triColor { 0.0, 0.5, 0.3, 0.8 };
        glm::vec4 lineColor { 1.0, 1.0, 1.0, 1.0 };
        int       nx { 32 };
        int       ny { 8 };
        int       nz { 8 }; // num of square grid

        float _Young { 20000 };
        float _rho { 400 };
        float _nu { 0.2 };
        float g { -0.1 };
        bool  _gravity_off { true };
        float _friction_ratio { 0.999 }; // 衰减系数
        bool  _friction_off { false };

        float _lambda = (_Young / _rho) * (_nu / ((1 + _nu) * (1 - 2 * _nu)));
        float _mu     = (_Young / _rho) / (2 * (1 + _nu)); // 拉梅系数
        float c       = std::sqrt(_Young / _rho);          // 声速
        float _maxVelocity { 0.0f };

        std::vector<Tetrahedron>         _tetrahetra;       // 存储所有四面体
        std::vector<Vertex>              _vertices;         // 存储所有顶点
        std::vector<std::pair<int, int>> _edges;            // 存储所有棱边，int为两个顶点的id
        std::vector<std::vector<int>>    _surfaceTriangles; // 存储所有在弹性体表面的三角形用于后续渲染

        inline int GetID(int i, int j, int k) {
            return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
        }

        inline glm::vec4 ColorMap(glm::vec3 v) {
            float ratio = std::min(glm::length(v) / (0.2f * c), 1.0f);
            return glm::vec4(1.0f, 1.0f, 1.0f - ratio, 0.8f);
        }

        void initialize();

        void AdvanceTetSystem(float const dt);

        void SimulateTimestep(float const dt) { // 细分dt是为了保证CFL条件满足
            int   numSubSteps = 3;
            float sdt         = dt / numSubSteps;

            for (int step { 0 }; step < numSubSteps; step++) {
                AdvanceTetSystem(sdt);
            }
        }
    };
} // namespace VCX::Labs::FEM