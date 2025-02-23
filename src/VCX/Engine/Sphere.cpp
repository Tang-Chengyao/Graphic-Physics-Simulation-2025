#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Sphere.h"

namespace VCX::Engine{
    const float PI = 3.1415926f;

    Sphere::Sphere(int precision, float radius) {
        init(precision,radius);
    }

    float Sphere::toRadians(float degrees) {
        return (degrees * PI) / 180;
    }

    void Sphere::init(int precision, float radius) {
        int mNumVertices = (precision + 1) * (precision + 1);
        int mNumIndices = precision * precision * 6;
        Positions.resize(mNumVertices);
        Normals.resize(mNumVertices);
        TexCoords.resize(mNumVertices);
        Indices.resize(mNumIndices);

        for (int i = 0; i <= precision; i++) {
            for (int j = 0; j <= precision; j++) {
                float y = static_cast<float>(cos(
                    toRadians(180.0f - i * 180.0f / (float) precision)));
                float tmp = toRadians(j * 360 / (float) precision);
                float x = static_cast<float>(-cos(tmp) * abs(cos(asin(y))));
                float z = static_cast<float>(sin(tmp) * abs(cos(asin(y))));
                int idx = i * (precision + 1) + j;
                Positions[idx] = radius * glm::vec3(x, y, z);
                TexCoords[idx] = glm::vec2(j / (float) precision, 
                    i / (float) precision);
                Normals[idx] = glm::vec3(x, y, z);
            }
        }

        for (int i = 0; i < precision; i++) {
            for (int j = 0; j < precision; j++) {
                int base = 6 * (i * precision + j);
                Indices[base] = i * (precision + 1) + j;
                Indices[base + 1] = i * (precision + 1) + j + 1;
                Indices[base + 2] = (i + 1) * (precision + 1) + j;
                Indices[base + 3] = i * (precision + 1) + j + 1;
                Indices[base + 4] = (i + 1) * (precision + 1) + j + 1;
                Indices[base + 5] = (i + 1) * (precision + 1) + j;
            }
        }
    }
}

