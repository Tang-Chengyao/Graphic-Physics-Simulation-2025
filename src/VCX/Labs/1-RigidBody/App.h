#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/1-RigidBody/CaseMultipleBox.h"
#include "Labs/1-RigidBody/CaseSingleBox.h"
#include "Labs/1-RigidBody/CaseTwoBox.h"
#include "Labs/1-RigidBody/Wall.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingleBox   _caseSingleBox;
        CaseTwoBox      _caseTwoBox;
        CaseMultipleBox _caseMultipleBox;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseSingleBox, _caseTwoBox, _caseMultipleBox };

    public:
        App();

        void OnFrame() override;
    };
}
