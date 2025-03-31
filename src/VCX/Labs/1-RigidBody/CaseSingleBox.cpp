#include "Labs/1-RigidBody/CaseSingleBox.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::RigidBody {

    CaseSingleBox::CaseSingleBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _I = _mass / 12.0f * glm::mat3(_dim.y * _dim.y + _dim.z * _dim.z, 0.0f, 0.0f, 0.0f, _dim.x * _dim.x + _dim.z * _dim.z, 0.0f, 0.0f, 0.0f, _dim.x * _dim.x + _dim.y * _dim.y);
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseSingleBox::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
        }
        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Position and Orientation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Position: (%.2f, %.2f, %.2f)", _center.x, _center.y, _center.z);
            ImGui::Text("Orientation: (%.2f, %.2f, %.2f, %.2f)", _q.w, _q.x, _q.y, _q.z);
            ImGui::SliderFloat3("Velocity", glm::value_ptr(_velocity), -5.0f, 5.0f);
            ImGui::SliderFloat3("Angular Veloity", glm::value_ptr(_angularVelocity), -5.0f, 5.0f);
        }
        if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Force Magnitude", &_forceMagnitude, 0.1f, 20.0f);
        }
        ImGui::Spacing();
        if (_isPaused) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f)); // 绿色
            if (ImGui::Button("Continue", ImVec2(120, 40))) {
                _isPaused = false;
            }
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.0f, 0.0f, 1.0f)); // 红色
            if (ImGui::Button("Stop", ImVec2(120, 40))) {
                _isPaused = true;
            }
            ImGui::PopStyleColor();
        }
    }

    void CaseSingleBox::UpdatePhysics(float dt) {
        glm::vec3 delta_velocity { 0.0f, 0.0f, 0.0f };
        glm::vec3 delta_angularVelocity { 0.0f, 0.0f, 0.0f };
        delta_velocity += _force / _mass * dt;
        glm::mat3 R           = glm::mat3_cast(_q);
        glm::mat3 I_world     = R * _I * glm::transpose(R);
        glm::mat3 I_world_inv = glm::inverse(I_world);
        delta_angularVelocity += dt * I_world_inv * (_torque - glm::cross(_angularVelocity, I_world * _angularVelocity));

        _velocity += delta_velocity;
        _angularVelocity += delta_angularVelocity;
        _center += dt * _velocity;
        glm::quat rotationQuat = glm::exp(0.5f * dt * glm::quat(0.0f, _angularVelocity));
        _q                     = glm::normalize(rotationQuat * _q);
    }

    Common::CaseRenderResult CaseSingleBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        if (! _isPaused) {
            UpdatePhysics(Engine::GetDeltaTime());
        }

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition;
        glm::mat3              rotationMatrix = glm::mat3_cast(_q); // 四元数转旋转矩阵

        std::vector<glm::vec3> localVerts = {
            { -_dim[0] / 2,  _dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2,  _dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2,  _dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2,  _dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2, -_dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2, -_dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2, -_dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2, -_dim[1] / 2, -_dim[2] / 2 }
        };

        for (const auto & localVert : localVerts) {
            VertsPosition.push_back(_center + rotationMatrix * localVert);
        }

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec4(1.f, 1.f, 1.f, 1.0f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

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

    void CaseSingleBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        ImGuiIO & io = ImGui::GetIO();
        // 更新按键状态（顺序：W, A, S, D, Space, Ctrl）
        _keyStates[0] = io.KeysDown[ImGuiKey_W];
        _keyStates[1] = io.KeysDown[ImGuiKey_A];
        _keyStates[2] = io.KeysDown[ImGuiKey_S];
        _keyStates[3] = io.KeysDown[ImGuiKey_D];
        _keyStates[4] = io.KeysDown[ImGuiKey_Space];
        _keyStates[5] = io.KeyCtrl;

        glm::vec3 forceDirection(0.0f);
        if (_keyStates[0]) forceDirection.z -= 1.0f; // W: 前
        if (_keyStates[1]) forceDirection.x -= 1.0f; // A: 左
        if (_keyStates[2]) forceDirection.z += 1.0f; // S: 后
        if (_keyStates[3]) forceDirection.x += 1.0f; // D: 右
        if (_keyStates[4]) forceDirection.y += 1.0f; // Space: 上
        if (_keyStates[5]) forceDirection.y -= 1.0f; // Ctrl: 下

        // 坐标系转换
        if (glm::length(forceDirection) > 0.0f) {
            forceDirection    = glm::normalize(forceDirection);
            glm::mat3 viewRot = glm::mat3(_camera.GetViewMatrix());
            _force            = viewRot * forceDirection * _forceMagnitude;
        } else {
            _force = glm::vec3(0.0f);
        }
    }

    void CaseSingleBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        _center += mouseDelta * movingScale;
    }

} // namespace VCX::Labs::RigidBody
