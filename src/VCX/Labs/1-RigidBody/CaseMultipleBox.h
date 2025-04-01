#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/1-RigidBody/Wall.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {

    class CaseMultipleBox : public Common::ICase {
    public:
        CaseMultipleBox();

        virtual std::string_view const GetName() override { return "Two Boxes Collision"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void UpdatePhysics(float dt);
        void HandleCollisionBoxBox(Box & box1, Box & box2);
        void HandleCollisionWallBox(const Wall & wall, Box & box);

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera {
                           .Eye    = glm::vec3(-15.0f, 0.0f, 15.0f),
                           .Target = glm::vec3 { 0.0f, 0.0f, 0.0f },
                           .Up     = glm::vec3 { 0.0f, 0.0f, 1.0f },
                           .ZFar   = 1000.0f
        };
        Common::OrbitCameraManager _cameraManager;
        std::vector<Wall>          _walls;
        std::vector<Box>           _boxes;
        int                        _numBoxes { 50 };
        float                      _gravity { 9.8f };
        float                      _muN { 0.8f };
        float                      _muT { 0.2f };
        float                      _c { 0.8f };
        float                      _energy { 0.0f };
        bool                       _isPaused { true };
    };
} // namespace VCX::Labs::RigidBody