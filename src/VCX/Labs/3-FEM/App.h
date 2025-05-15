#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/FEM.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFEM _caseFEM;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFEM };

    public:
        App();

        void OnFrame() override;
    };
}