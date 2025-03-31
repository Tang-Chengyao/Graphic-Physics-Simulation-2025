#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {

    class CaseSingleBox : public Common::ICase {
    public:
        CaseSingleBox();

        virtual std::string_view const GetName() override { return "A Single Rigid Box"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void UpdatePhysics(float dt);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        glm::vec3                           _center { 0.f, 0.f, 0.f };
        glm::vec3                           _dim { 1.f, 2.f, 3.f };
        glm::vec4                           _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255, 0.5f };
        glm::quat                           _q { 1.0f, 0.0f, 0.0f, 0.f };
        glm::vec3                           _velocity { 0.f, 0.f, 0.f };
        glm::vec3                           _angularVelocity { 0.0f, 0.0f, 0.0f };
        glm::vec3                           _force { 0.0f, 0.0f, 0.0f };
        glm::vec3                           _torque { 0.0f, 0.0f, 0.0f };
        float                               _mass { 1.0f };
        glm::mat3                           _I;
        float                               _forceMagnitude = 5.0f;
        std::vector<bool>                   _keyStates      = std::vector<bool>(6, false);
        bool                                _isPaused { true };
    };
} // namespace VCX::Labs::RigidBody
