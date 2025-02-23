#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/0-GettingStarted/CaseBox.h"
#include "Labs/0-GettingStarted/CaseFixed.h"
#include "Labs/0-GettingStarted/CaseFluid.h"
#include "Labs/0-GettingStarted/CaseMassSpring.h"
#include "Labs/0-GettingStarted/CaseResizable.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::GettingStarted {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFixed      _caseFixed;
        CaseResizable  _caseResizable;
        CaseBox        _caseBox;
        CaseMassSpring _caseMassSpring;
        CaseFluid      _casefluid;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFixed, _caseResizable, _caseBox, _caseMassSpring, _casefluid };

    public:
        App();

        void OnFrame() override;
    };
}
