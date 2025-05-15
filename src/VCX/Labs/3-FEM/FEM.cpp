#include "Labs/3-FEM/FEM.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <GLFW/glfw3.h>
#include <ranges>

namespace VCX::Labs::FEM {
    CaseFEM::CaseFEM():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _verticesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec4>("color", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points),
        _linesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _trianglesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _wallItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _arrowItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles) {
        _cameraManager.AutoRotate = false;
        ResetSystem();

        std::vector<std::uint32_t> line_index;
        for (std::uint32_t i { 0 }; i < static_cast<uint32_t>(2 * _tetsystem._edges.size()); i++) {
            line_index.push_back(i);
        }
        _linesItem.UpdateElementBuffer(line_index);

        std::vector<std::uint32_t> tri_index;
        for (std::uint32_t i { 0 }; i < static_cast<uint32_t>(3 * _tetsystem._surfaceTriangles.size()); i++) {
            tri_index.push_back(i);
        }
        _trianglesItem.UpdateElementBuffer(tri_index);

        std::vector<std::uint32_t> wall_index { 0, 1, 2, 0, 2, 3 };
        _wallItem.UpdateElementBuffer(wall_index);
    }

    void CaseFEM::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button(_tetsystem._gravity_off ? "Open Gravity" : "Shut down Gravity")) _tetsystem._gravity_off = ! _tetsystem._gravity_off;
            ImGui::SameLine();
            if (ImGui::Button(_tetsystem._friction_off ? "Open Friction" : "Shut down Friction")) _tetsystem._friction_off = ! _tetsystem._friction_off;
            ImGui::SliderFloat("Poisson's Ratio: ", &_tetsystem._nu, -1.0f, 0.5f, "%.2f");
            ImGui::Text("Press H(-x) J(-y) K(+y) L(+x) to add force to the right end face.");
            ImGui::Text("Max Velocity: %.3f c (speed of sound)", _tetsystem._maxVelocity / _tetsystem.c);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            if (ImGui::Button("Rendering Mode 1 (Surfaces)")) _render_mode = 0;
            if (ImGui::Button("Rendering Mode 2 (Lines)")) _render_mode = 1;
            if (ImGui::Button("Rendering Mode 3 (Points)")) _render_mode = 2;
        }
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {
            _tetsystem.SimulateTimestep(Engine::GetDeltaTime());
        }

        // 墙的渲染
        _wallItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_wallPositions));

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glPointSize(_vertexSize);
        glLineWidth(_lineWidth);

        _program.GetUniforms().SetByName("useUniformColor", 1);
        _program.GetUniforms().SetByName("u_Color", glm::vec4(0.5f, 0.5f, 0.5f, 0.5f));
        _wallItem.Draw({ _program.Use() });

        if (_render_mode == 0) {
            std::vector<glm::vec3> triVertexPositions;
            for (const auto tri : _tetsystem._surfaceTriangles) {
                triVertexPositions.push_back(_tetsystem._vertices[tri[0]].pos);
                triVertexPositions.push_back(_tetsystem._vertices[tri[1]].pos);
                triVertexPositions.push_back(_tetsystem._vertices[tri[2]].pos);
            }
            _trianglesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(triVertexPositions));
            _program.GetUniforms().SetByName("useUniformColor", 1);
            _program.GetUniforms().SetByName("u_Color", _triangleColor);
            _trianglesItem.Draw({ _program.Use() });
        }
        if (_render_mode == 1) {
            // 棱边渲染对象 更新缓冲区
            std::vector<glm::vec3> lineVerticesPositions;
            for (const auto line : _tetsystem._edges) {
                lineVerticesPositions.push_back(_tetsystem._vertices[line.first].pos);
                lineVerticesPositions.push_back(_tetsystem._vertices[line.second].pos);
            }
            _linesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(lineVerticesPositions));
            _program.GetUniforms().SetByName("useUniformColor", 1);
            _program.GetUniforms().SetByName("u_Color", _lineColor);
            _linesItem.Draw({ _program.Use() });
        }
        if (_render_mode == 2) {
            // 顶点渲染对象 更新缓冲区
            std::vector<glm::vec3> verticesPositions;
            std::vector<glm::vec4> verticesColors;
            for (const auto vertex : _tetsystem._vertices) {
                verticesPositions.push_back(vertex.pos);
                verticesColors.push_back(vertex.color);
            }
            _verticesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(verticesPositions));
            _verticesItem.UpdateVertexBuffer("color", Engine::make_span_bytes<glm::vec4>(verticesColors));
            _program.GetUniforms().SetByName("useUniformColor", 0);
            _verticesItem.Draw({ _program.Use() });
        }
        if (_showArrow) {
            _program.GetUniforms().SetByName("useUniformColor", 1);
            _program.GetUniforms().SetByName("u_Color", glm::vec4(1.0f, 0.0f, 0.0f, 0.8f));
            _arrowItem.Draw({ _program.Use() });
        }

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFEM::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        // 捕获键盘事件
        if (ImGui::IsKeyPressed(ImGuiKey_L)) {
            ProcessKeyInput(GLFW_KEY_L, GLFW_PRESS);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_H)) {
            ProcessKeyInput(GLFW_KEY_H, GLFW_PRESS);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_J)) {
            ProcessKeyInput(GLFW_KEY_J, GLFW_PRESS);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_K)) {
            ProcessKeyInput(GLFW_KEY_K, GLFW_PRESS);
        }

        if (ImGui::IsKeyReleased(ImGuiKey_L) || ImGui::IsKeyReleased(ImGuiKey_H) || ImGui::IsKeyReleased(ImGuiKey_J) || ImGui::IsKeyReleased(ImGuiKey_K)) {
            _showArrow = false; // 松开按键时隐藏箭头
        }
    }

    void CaseFEM::ProcessKeyInput(int key, int action) {
        if (action != GLFW_PRESS) return; // 只在按下时触发

        glm::vec3 impulse(0.0f);
        glm::vec3 offset(0.0f);
        glm::vec3 direction(1.0f, 0.0f, 0.0f);
        char      type { 'x' };
        switch (key) {
        case GLFW_KEY_L:
            impulse = glm::vec3(impulseMagnitude, 0.0f, 0.0f);
            break;

        case GLFW_KEY_H:
            impulse = glm::vec3(-impulseMagnitude, 0.0f, 0.0f);
            offset += _arrowScale * glm::vec3(1.0f, 0.0f, 0.0f);
            direction = glm::vec3(-1.0f, 0.0f, 0.0f);
            break;

        case GLFW_KEY_J:
            impulse   = glm::vec3(0.0f, -impulseMagnitude, 0.0f);
            direction = glm::vec3(0.0f, -1.0f, 0.0f);
            type      = 'y';
            break;

        case GLFW_KEY_K:
            impulse   = glm::vec3(0.0f, impulseMagnitude, 0.0f);
            direction = glm::vec3(0.0f, 1.0f, 0.0f);
            type      = 'y';
            break;
        }

        glm::vec3 rightEndPos(0.0f);
        int       rightEndVertexCount = 0;
        for (auto & vertex : _tetsystem._vertices) {
            int i = vertex.id / ((_tetsystem.ny + 1) * (_tetsystem.nz + 1));
            if (i == _tetsystem.nx) { // 该顶点在最右边（自由表面处）
                vertex.v += impulse / vertex.mass;
                rightEndPos += vertex.pos;
                rightEndVertexCount++;
            }
        }

        if (rightEndVertexCount > 0) {
            rightEndPos /= rightEndVertexCount; // 计算右端面的中心位置
            UpdateArrow(rightEndPos + offset, direction, type);
            _showArrow = true;
        }
    }

    void CaseFEM::UpdateArrow(const glm::vec3 & startPos, const glm::vec3 & direction, char type) {
        _arrowVertices.clear();
        std::vector<std::uint32_t> arrow_index;
        glm::vec3                  endPos = startPos + direction * _arrowScale;

        if (type == 'x') {
            arrow_index = { 0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 0, 8 };
            // 圆锥底面
            const int segments { 8 };
            for (int i { 0 }; i < segments; i++) {
                float     angle = 2.0f * glm::pi<float>() * i / segments;
                glm::vec3 offset(0.0f, cos(angle), sin(angle));
                _arrowVertices.push_back(startPos + 0.4f * offset * _arrowScale);
            }
            // 圆锥顶点
            _arrowVertices.push_back(endPos);
        } else if (type == 'y') {
            arrow_index = { 0, 1, 3, 0, 2, 3 };
            _arrowVertices.push_back(startPos);
            _arrowVertices.push_back(startPos + 0.4f * glm::vec3(0.0f, 0.0f, 1.0f) * _arrowScale);
            _arrowVertices.push_back(startPos - 0.4f * glm::vec3(0.0f, 0.0f, 1.0f) * _arrowScale);
            _arrowVertices.push_back(endPos);
        }

        _arrowItem.UpdateElementBuffer(arrow_index);
        _arrowItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_arrowVertices));
    }

    void CaseFEM::ResetSystem() {
        _tetsystem.initialize();
        _camera.Eye    = glm::vec3(18.0f, -9.0f, 8.0f);
        _camera.Target = glm::vec3(4.0f, 2.0f, -2.0f);
        _camera.Up     = glm::vec3(0.0f, 0.0f, 1.0f);
        _camera.Fovy   = 45.0f;
        _cameraManager.Save(_camera);
    }
} // namespace VCX::Labs::FEM