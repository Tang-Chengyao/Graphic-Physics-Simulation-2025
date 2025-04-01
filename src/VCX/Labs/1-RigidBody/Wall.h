#pragma once

#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Wall {
        int                                 id;
        glm::vec4                           color;
        glm::vec3                           position;
        glm::vec2                           dim;
        glm::vec3                           tangent1;
        glm::vec3                           tangent2;
        glm::vec3                           normal;
        std::vector<glm::vec3>              VerticesPosition;
        Engine::GL::UniqueIndexedRenderItem TriangleItem;
        Engine::GL::UniqueIndexedRenderItem LineItem;

        Wall(int id, glm::vec3 position, glm::vec2 dim, glm::vec3 t1, glm::vec3 t2, glm::vec4 color = glm::vec4(0.5f, 0.5f, 0.5f, 0.3f)):
            id(id),
            color(color),
            position(position),
            dim(dim),
            tangent1(glm::normalize(t1)),
            tangent2(glm::normalize(t1)),
            normal(glm::normalize(glm::cross(t1, t2))),
            VerticesPosition({
                position + dim.x / 2.0f * t1 + dim.y / 2.0f * t2,
                position - dim.x / 2.0f * t1 + dim.y / 2.0f * t2,
                position - dim.x / 2.0f * t1 - dim.y / 2.0f * t2,
                position + dim.x / 2.0f * t1 - dim.y / 2.0f * t2,
            }),
            TriangleItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
            //  1-----0
            //  |     |
            //  |     |
            //  2-----3
            const std::vector<std::uint32_t> wallLineIndex { 0, 1, 1, 2, 2, 3, 3, 0 };
            const std::vector<std::uint32_t> wallTriIndex { 0, 1, 2, 0, 2, 3 };
            TriangleItem.UpdateElementBuffer(wallTriIndex);
            LineItem.UpdateElementBuffer(wallLineIndex);
        }
    };
}