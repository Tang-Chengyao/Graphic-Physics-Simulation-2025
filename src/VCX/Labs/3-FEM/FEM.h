#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Labs/3-FEM/TetSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::FEM {

    class CaseFEM : public Common::ICase {
    public:
        CaseFEM();

        virtual std::string_view const GetName() override { return "FEM Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        // 键盘交互函数
        void ProcessKeyInput(int key, int action);
        void UpdateArrow(const glm::vec3 & startPos, const glm::vec3 & direction, char type);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(4.0f, -10.0f, 1.0f), .Target = glm::vec3(4.0f, 0.0f, 0.0f), .Up = glm::vec3(0.0f, 0.0f, 1.0f) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueRenderItem        _verticesItem;  // 渲染顶点
        Engine::GL::UniqueIndexedRenderItem _linesItem;     // 渲染棱边
        Engine::GL::UniqueIndexedRenderItem _trianglesItem; // 渲染三角形面
        TetSystem                           _tetsystem;
        int                                 _render_mode { 2 }; // 0：渲染表面；1：渲染线；2：渲染点。
        bool                                _stopped { false };
        float                               _vertexSize { 5 };
        float                               _lineWidth { 1 };
        glm::vec4                           _vertexColor { 0.0f, 0.5f, 0.2f, 1.0f };
        glm::vec4                           _lineColor { 1.0f, 1.0f, 1.0f, 1.0f };
        glm::vec4                           _triangleColor { 0.5, 0.1f, 0.5f, 0.8f };
        std::vector<glm::vec3>              _wallPositions {
            glm::vec3(0.0f, -5.0f, -5.0f),
            glm::vec3(0.0f, 5.0f, -5.0f),
            glm::vec3(0.0f, 5.0f, 5.0f),
            glm::vec3(0.0f, -5.0f, 5.0f)
        };
        Engine::GL::UniqueIndexedRenderItem _wallItem; // 渲染固定墙面
        float                               impulseMagnitude { 2.0f };

        std::vector<glm::vec3>              _arrowVertices;
        Engine::GL::UniqueIndexedRenderItem _arrowItem;
        float                               _arrowScale { 2.0f };
        bool                                _showArrow { false };

        void ResetSystem();
    };
} // namespace VCX::Labs::FEM