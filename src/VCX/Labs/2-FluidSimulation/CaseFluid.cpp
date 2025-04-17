#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>
#include <spdlog/spdlog.h>

namespace VCX::Labs::Fluid {
    const std::vector<glm::vec3> vertex_pos = {
        glm::vec3(-0.5f, -0.5f, -0.5f),
        glm::vec3(0.5f, -0.5f, -0.5f),
        glm::vec3(0.5f, 0.5f, -0.5f),
        glm::vec3(-0.5f, 0.5f, -0.5f),
        glm::vec3(-0.5f, -0.5f, 0.5f),
        glm::vec3(0.5f, -0.5f, 0.5f),
        glm::vec3(0.5f, 0.5f, 0.5f),
        glm::vec3(-0.5f, 0.5f, 0.5f)
    };
    const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index

    CaseFluid::CaseFluid(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/sphere_phong.vert"), Engine::GL::SharedShader("assets/shaders/phong.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap", 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap", 2);
        _lineprogram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);
        ResetSystem();
        _sphere = Engine::Model { Engine::Sphere(6, _simulation.m_particleRadius), 0 };

        // 初始化障碍物球模型
        _obstacleSphere = Engine::Model { Engine::Sphere(12, 0.2f), 0 };
    }

    void CaseFluid::OnSetupPropsUI() {
        if (ImGui::Button("Reset System"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::Spacing();
        // 新增补偿漂移开关按钮
        ImGui::Checkbox("Compensate Drift", &_simulation.compensateDrift); // 使用 Checkbox
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Enable/disable velocity drift compensation during FLIP-PIC blending.");
        ImGui::SliderFloat("Rest Density", &_simulation.m_particleRestDensity, 0.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Compensate Drift k", &_simulation.ko, 0.0f, 2.0f, "%.2f");
        ImGui::Spacing();
        ImGui::SliderFloat("FLIP Ratio", &_simulation.m_fRatio, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Time Step", &_simulation.dt, 0.001f, 0.05f, "%.3f");
    }

    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        static bool firstRender = true;
        if (firstRender) {
            firstRender            = false;
            _sceneObject.Camera.Up = glm::vec3(0.0f, 0.0f, 1.0f); // 强制设置 Up
            _cameraManager.Save(_sceneObject.Camera);
        }

        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }

        if (! _stopped) _simulation.SimulateTimestep(_simulation.dt);

        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());

        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale", _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn", _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess", _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder", _attenuationOrder);
            _program.GetUniforms().SetByName("u_BumpMappingBlend", _bumpMappingPercent * .01f);
        }

        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        Rendering::ModelObject m        = Rendering::ModelObject(_sphere, _simulation.m_particlePos);
        auto const &           material = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(), material.MetaSpec.Use(), material.Height.Use(), _program.Use() }, _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres);

        // 渲染障碍物球（红色）
        auto program = _program.Use();
        _program.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());

        // 设置红色材质
        _program.GetUniforms().SetByName("u_AmbientScale", 0.5f);
        _program.GetUniforms().SetByName("u_Shininess", 32.0f);
        _program.GetUniforms().SetByName("u_UseGammaCorrection", 1);
        _program.GetUniforms().SetByName("u_AttenuationOrder", 2);
        _program.GetUniforms().SetByName("u_BumpMappingBlend", 0.0f);

        // 使用纯红色（禁用纹理）
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, 0);

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.0f, 0.0f)); // 红色

        // 创建障碍物球的 ModelObject 实例
        std::vector<glm::vec3> obstaclePositions = { _simulation.obstaclePos }; // 障碍物球的位置
        Rendering::ModelObject obstacleModel(_obstacleSphere, obstaclePositions);

        // 获取材质（这里假设使用默认材质）
        auto const & material1 = _sceneObject.Materials[0];

        // 渲染障碍物球
        obstacleModel.Mesh.Draw({ material1.Albedo.Use(), material1.MetaSpec.Use(), material1.Height.Use(), _program.Use() }, _obstacleSphere.Mesh.Indices.size(), 0,
                                1); // 障碍物球只有一个实例
        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    // 在 OnProcessInput 函数中添加鼠标交互
    void CaseFluid::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
        // 获取输入状态
        auto & io = ImGui::GetIO();

        // 设置障碍物的速度
        float     speed = 0.1f; // 设置移动速度
        glm::vec3 movement(0.0f);

        // 处理键盘输入
        if (io.KeysDown[ImGuiKey_UpArrow]) movement.y += speed;
        if (io.KeysDown[ImGuiKey_DownArrow]) movement.y -= speed;
        if (io.KeysDown[ImGuiKey_LeftArrow]) movement.x -= speed;
        if (io.KeysDown[ImGuiKey_RightArrow]) movement.x += speed;
        if (io.KeysDown[ImGuiKey_U]) movement.z += speed;
        if (io.KeysDown[ImGuiKey_D]) movement.z -= speed;

        // 更新障碍物位置
        _simulation.obstaclePos += movement * io.DeltaTime;

        // 确保障碍物位置在容器范围内
        _simulation.obstaclePos.x = glm::clamp(_simulation.obstaclePos.x, _simulation.xmin + _simulation.m_h, _simulation.xmax - _simulation.m_h);
        _simulation.obstaclePos.y = glm::clamp(_simulation.obstaclePos.y, _simulation.ymin + _simulation.m_h, _simulation.ymax - _simulation.m_h);
        _simulation.obstaclePos.z = glm::clamp(_simulation.obstaclePos.z, _simulation.zmin + _simulation.m_h, _simulation.zmax - _simulation.m_h);

        // 更新障碍物速度（基于移动方向）
        _simulation.obstacleVel = movement;
    }

    void CaseFluid::ResetSystem() {
        _simulation.setupScene(_res);

        _sceneObject.Camera.Eye    = glm::vec3(0.0f, -2.0f, 1.0f); // 相机位置（可根据需要调整）
        _sceneObject.Camera.Target = glm::vec3(0.0f, 0.0f, 0.0f);  // 焦点（看向原点）
        _sceneObject.Camera.Up     = glm::vec3(0.0f, 0.0f, 1.0f);  // up 向量设为 Z 轴
        _sceneObject.Camera.Fovy   = 45.0f;                        // 视野角度
        _cameraManager.Save(_sceneObject.Camera);                  // 保存相机状态
    }
} // namespace VCX::Labs::Fluid