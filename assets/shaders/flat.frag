#version 410 core

layout(location = 0) in  vec3 v_Position;

layout(location = 0) out vec4 f_Color;

uniform vec4  u_Color;

void main() {
    f_Color = u_Color;
}
