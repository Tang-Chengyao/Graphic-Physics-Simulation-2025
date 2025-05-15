#version 410 core

// 输入：来自顶点着色器的顶点位置（location=0）
layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec4 v_Color;

// 输出：最终像素颜色（location=0）
layout(location = 0) out vec4 f_Color;

// Uniform变量（由CPU传入）
uniform vec4  u_Color;
uniform int  useUniformColor = 1; // 控制是否用单一颜色的开关，默认为是

void main() {
    f_Color = bool(useUniformColor) ? u_Color : v_Color;
}
