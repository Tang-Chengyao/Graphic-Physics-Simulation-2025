#pragma once

#include "Labs/3-FEM/Vertex.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct Tetrahedron {
        int                   id;
        glm::mat3             E_inverse; // Reference state的E^-1矩阵
        std::vector<Vertex *> vertices;  // 四个顶点的

        Tetrahedron():
            id(0), E_inverse(glm::mat3(1.0f)) { vertices.resize(4); }
        Tetrahedron(int id):
            id(id),
            E_inverse(glm::mat3(1.0f)) {
            vertices.resize(4);
        }
    };
}
