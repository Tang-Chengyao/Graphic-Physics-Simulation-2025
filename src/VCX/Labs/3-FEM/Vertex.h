#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct Vertex {
        int       id;
        glm::vec3 pos;
        glm::vec3 v;
        glm::vec3 f;    // 形变后的力
        float     mass; // 形变后的质量
        int       type; // type = 3: at the corner; type = 2: on the edge; type = 1: on the surface; 0: inside the volumn;
        glm::vec4 color;

        // 默认构造函数
        Vertex():
            id(0), pos(glm::vec3(0)), v(glm::vec3(0)), f(glm::vec3(0)), mass(0.0f), type(0), color(glm::vec4(0)) {}

        Vertex(int id, glm::vec3 pos):
            id(id), pos(pos), v(glm::vec3(0)), f(glm::vec3(0)), mass(0.0f), type(0), color(glm::vec4(0)) {}
    };
}