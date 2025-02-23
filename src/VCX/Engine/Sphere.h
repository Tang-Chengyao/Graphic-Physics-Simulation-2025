#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "SurfaceMesh.h"

namespace VCX::Engine{
    class Sphere:public SurfaceMesh{
    public:
        Sphere(int precision,float radius);

        std::vector<uint32_t> GetIndices() { return Indices; }

        std::vector<glm::vec2> GetTexCoords() { return TexCoords; }

        std::vector<glm::vec3> GetVertices() { return Positions; }

        std::vector<glm::vec3> GetNormals() { return Normals; }

    private:

        void init(int precision, float radius);

        float toRadians(float degrees);
    };
}