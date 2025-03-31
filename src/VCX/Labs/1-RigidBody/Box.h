#pragma once

#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Box {
        int                                 id;
        glm::vec4                           color;
        float                               mass;
        glm::vec3                           dim;
        glm::mat3                           I;
        glm::vec3                           position;
        glm::quat                           q;
        glm::vec3                           v;
        glm::vec3                           w;
        std::vector<glm::vec3>              VerticesPosition;
        Engine::GL::UniqueIndexedRenderItem TriangleItem;
        Engine::GL::UniqueIndexedRenderItem LineItem;

        Box(int id, float mass, glm::vec3 dim, glm::vec3 position, glm::vec4 color = glm::vec4(1.0f), glm::quat q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)):
            id(id),
            color(color),
            mass(mass),
            dim(dim),
            I(1.0f / 12.0f * mass * glm::mat3(dim.y * dim.y + dim.z * dim.z, 0.0f, 0.0f, 0.0f, dim.x * dim.x + dim.z * dim.z, 0.0f, 0.0f, 0.0f, dim.x * dim.x + dim.y * dim.y)),
            position(position),
            q(glm::normalize(q)),
            v(glm::vec3(0.0f)),
            w(glm::vec3(0.0f)),
            VerticesPosition({ glm::vec3(-dim.x, dim.y, dim.z) / 2.0f, glm::vec3(dim.x, dim.y, dim.z) / 2.0f, glm::vec3(dim.x, dim.y, -dim.z) / 2.0f, glm::vec3(-dim.x, dim.y, -dim.z) / 2.0f, glm::vec3(-dim.x, -dim.y, dim.z) / 2.0f, glm::vec3(dim.x, -dim.y, dim.z) / 2.0f, glm::vec3(dim.x, -dim.y, -dim.z) / 2.0f, glm::vec3(-dim.x, -dim.y, -dim.z) / 2.0f }),
            TriangleItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {}
    };
} // namespace VCX::Labs::RigidBody