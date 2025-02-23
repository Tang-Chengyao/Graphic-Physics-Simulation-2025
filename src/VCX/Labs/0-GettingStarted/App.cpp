#include "Assets/bundled.h"
#include "Labs/0-GettingStarted/App.h"

namespace VCX::Labs::GettingStarted {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _casefluid({ Assets::ExampleScene::Fluid }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
