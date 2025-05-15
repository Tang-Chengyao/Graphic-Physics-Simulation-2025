#include "TetSystem.h"
#include <iostream>

// 重载函数：输出 glm::vec3 向量
void printMatrix(const glm::vec3 & vec) {
    std::cout << vec.x << " " << vec.y << " " << vec.z << "\n";
}

using namespace VCX::Labs::FEM;

void TetSystem::initialize() {
    // 预先计算顶点和四面体的数量
    const int vertexCount      = (nx + 1) * (ny + 1) * (nz + 1);
    const int tetrahedronCount = nx * ny * nz * 6;

    // 直接分配内存并清空
    _vertices.clear();
    _vertices.resize(vertexCount); // 直接分配顶点内存
    _tetrahetra.clear();
    _tetrahetra.resize(tetrahedronCount); // 直接分配四面体内存
    _edges.clear();
    std::vector<std::vector<bool>> visited(vertexCount, std::vector<bool>(vertexCount, false)); // 用于初始化_edges的访问二维向量
    _surfaceTriangles.clear();

    float hx = Lx / nx;
    float hy = Ly / ny;
    float hz = Lz / nz;

    // 初始化所有顶点
    for (int i = 0; i <= nx; i++) {
        for (int j = 0; j <= ny; j++) {
            for (int k = 0; k <= nz; k++) {
                const int id  = GetID(i, j, k);
                _vertices[id] = Vertex(id, glm::vec3(i * hx, j * hy, k * hz));

                int typer = 0;
                if (i == 0 || i == nx) typer += 1;
                if (j == 0 || j == ny) typer += 1;
                if (k == 0 || k == nz) typer += 1;

                _vertices[id].type  = typer;
                _vertices[id].color = ColorMap(_vertices[id].v);
            }
        }
    }

    // 初始化四面体
    int TetId = 0;
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                // 获取当前立方体的8个顶点指针
                Vertex * v000 = &_vertices[GetID(i, j, k)];
                Vertex * v001 = &_vertices[GetID(i, j, k + 1)];
                Vertex * v010 = &_vertices[GetID(i, j + 1, k)];
                Vertex * v011 = &_vertices[GetID(i, j + 1, k + 1)];
                Vertex * v100 = &_vertices[GetID(i + 1, j, k)];
                Vertex * v101 = &_vertices[GetID(i + 1, j, k + 1)];
                Vertex * v110 = &_vertices[GetID(i + 1, j + 1, k)];
                Vertex * v111 = &_vertices[GetID(i + 1, j + 1, k + 1)];

                // 将一个立方体分割为6个四面体（直接通过索引赋值）
                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v001, v011, v111 }; // Tet1
                TetId++;

                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v010, v011, v111 }; // Tet2
                TetId++;

                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v001, v101, v111 }; // Tet3
                TetId++;

                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v100, v101, v111 }; // Tet4
                TetId++;

                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v010, v110, v111 }; // Tet5
                TetId++;

                _tetrahetra[TetId]          = Tetrahedron(TetId);
                _tetrahetra[TetId].vertices = { v000, v100, v110, v111 }; // Tet6
                TetId++;
            }
        }
    }

    // 初始化四面体的参考形状矩阵E_inverse
    for (auto & tet : _tetrahetra) {
        glm::vec3 X0 = tet.vertices[0]->pos;
        glm::vec3 X1 = tet.vertices[1]->pos;
        glm::vec3 X2 = tet.vertices[2]->pos;
        glm::vec3 X3 = tet.vertices[3]->pos;

        glm::mat3 Dm;
        Dm[0] = X1 - X0;
        Dm[1] = X2 - X0;
        Dm[2] = X3 - X0;

        tet.E_inverse = glm::inverse(Dm);
    }

    // 遍历四面体，完成对顶点质量的初始化
    for (const auto & tet : _tetrahetra) {           // 遍历四面体
        for (const auto vertex_ptr : tet.vertices) { // 遍历一个四面体的四个顶点指针
            vertex_ptr->mass += 1.0f / 4.0f;
        }
    }

    // 再次遍历四面体，完成对棱边列表_edges的初始化
    for (const auto tet : _tetrahetra) {
        for (int i { 0 }; i < 4; i++) {
            if (i < 3) {
                for (int j { i + 1 }; j < 4; j++) {
                    int id_i = tet.vertices[i]->id;
                    int id_j = tet.vertices[j]->id;
                    if (! visited[id_i][id_j]) {
                        _edges.emplace_back(std::pair<int, int>(id_i, id_j));
                        visited[id_i][id_j] = true;
                        visited[id_j][id_i] = true; // 1-2 和 2-1 实际上是一条边
                    }
                }
            }
        }
    }

    // 对表面三角形列表_surfaceTriangles的初始化
    // i = 0
    for (int j { 0 }; j < ny; j++) {
        for (int k { 0 }; k < nz; k++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(0, j, k), GetID(0, j, k + 1), GetID(0, j + 1, k + 1) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(0, j, k), GetID(0, j + 1, k), GetID(0, j + 1, k + 1) });
        }
    }
    // i = nx
    for (int j { 0 }; j < ny; j++) {
        for (int k { 0 }; k < nz; k++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(nx, j, k), GetID(nx, j, k + 1), GetID(nx, j + 1, k + 1) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(nx, j, k), GetID(nx, j + 1, k), GetID(nx, j + 1, k + 1) });
        }
    }
    // j = 0
    for (int i { 0 }; i < nx; i++) {
        for (int k { 0 }; k < nz; k++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, 0, k), GetID(i, 0, k + 1), GetID(i + 1, 0, k + 1) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, 0, k), GetID(i + 1, 0, k), GetID(i + 1, 0, k + 1) });
        }
    }
    // j = ny
    for (int i { 0 }; i < nx; i++) {
        for (int k { 0 }; k < nz; k++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, ny, k), GetID(i, ny, k + 1), GetID(i + 1, ny, k + 1) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, ny, k), GetID(i + 1, ny, k), GetID(i + 1, ny, k + 1) });
        }
    }
    // k = 0
    for (int i { 0 }; i < nx; i++) {
        for (int j { 0 }; j < ny; j++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, j, 0), GetID(i, j + 1, 0), GetID(i + 1, j + 1, 0) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, j, 0), GetID(i + 1, j, 0), GetID(i + 1, j + 1, 0) });
        }
    }
    // k = nz
    for (int i { 0 }; i < nx; i++) {
        for (int j { 0 }; j < ny; j++) {
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, j, nz), GetID(i, j + 1, nz), GetID(i + 1, j + 1, nz) });
            _surfaceTriangles.push_back(
                std::vector<int> { GetID(i, j, nz), GetID(i + 1, j, nz), GetID(i + 1, j + 1, nz) });
        }
    }

    std::cout << "size of _tethedra: " << _tetrahetra.size() << std::endl;
    std::cout << "size of _vertices: " << _vertices.size() << std::endl;
    std::cout << "size of _edges: " << _edges.size() << std::endl;
}

void TetSystem::AdvanceTetSystem(float dt) {
    // 清零所有顶点的力
    for (auto & vertex : _vertices) {
        vertex.f = glm::vec3(0);
    }
    // 遍历四面体，按StVK模型计算顶点受力，累加到每个顶点上
    for (auto & tet : _tetrahetra) {
        glm::vec3 x0 = tet.vertices[0]->pos;
        glm::vec3 x1 = tet.vertices[1]->pos;
        glm::vec3 x2 = tet.vertices[2]->pos;
        glm::vec3 x3 = tet.vertices[3]->pos;

        glm::mat3 Dm;
        Dm[0] = x1 - x0;
        Dm[1] = x2 - x0;
        Dm[2] = x3 - x0;
        // printMatrix(Dm);
        glm::mat3 F     = Dm * tet.E_inverse;
        glm::mat3 G     = 1.0f / 2.0f * (glm::transpose(F) * F - glm::mat3(1.0f));
        float     trace = G[0][0] + G[1][1] + G[2][2];
        glm::mat3 S     = 2 * _mu * G + _lambda * trace * glm::mat3(1.0f);
        glm::mat3 P     = F * S;
        glm::mat3 f123  = -P * glm::transpose(tet.E_inverse); // 已经约去了V_ref
        glm::vec3 f1    = f123[0];
        glm::vec3 f2    = f123[1];
        glm::vec3 f3    = f123[2];
        glm::vec3 f0    = -f1 - f2 - f3;

        tet.vertices[0]->f += f0;
        tet.vertices[1]->f += f1;
        tet.vertices[2]->f += f2;
        tet.vertices[3]->f += f3; // 把力累加到每个顶点的f属性中
    }
    // 显示欧拉更新速度和位置

    float maxV { 0.0f };
    for (auto & vertex : _vertices) {
        if (vertex.id > (ny * (nz + 1) + nz)) { // 边界条件：x=0固定
            vertex.v += dt * vertex.f / vertex.mass;
            if (! _gravity_off) {
                vertex.v += dt * glm::vec3(0.0f, 0.0f, g); // 重力
            }
            if (! _friction_off) {
                vertex.v *= _friction_ratio;
            }

            maxV         = std::max(maxV, glm::length(vertex.v));
            vertex.color = ColorMap(vertex.v);
            vertex.pos += dt * vertex.v;
        }
    }
    _maxVelocity = maxV;
}