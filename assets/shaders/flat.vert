#version 410 core

// 输入：顶点位置（来自顶点缓冲，location=0）
layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec4 a_Color;

// 输出：传递到片段着色器的顶点位置（location=0）
layout(location = 0) out vec3 v_Position;
layout(location = 1) out vec4 v_Color;

// Uniform变量（由CPU传入）
uniform mat4  u_Projection; // 投影矩阵
uniform mat4  u_View;       // 视图矩阵

void main() {
    v_Position  = a_Position; // 直接将顶点位置传递给片段着色器
    v_Color = a_Color; // 传递颜色到片段着色器
    gl_Position = u_Projection * u_View * vec4(v_Position, 1.); // 变换到裁剪空间
}
