#pragma once

#include <array>
#include <string_view>

namespace VCX::Assets {
    inline constexpr auto DefaultIcons {
        std::to_array<std::string_view>({
            "assets/images/vcl-logo-32x32.png",
            "assets/images/vcl-logo-48x48.png",
        })
    };

    inline constexpr auto DefaultFonts {
        std::to_array<std::string_view>({
            "assets/fonts/Ubuntu.ttf",
            "assets/fonts/UbuntuMono.ttf",
        })
    };
    inline constexpr auto ExampleScenes {
        std::to_array<std::string_view>({
            "assets/scenes/fluid/fluid.yaml",
        })
    };

    enum class ExampleScene {
        Fluid,
    };
}
