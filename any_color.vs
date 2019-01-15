#version 330

layout(location = 0) in vec4 position;
layout(location = 4) in vec4 color;

uniform mat4 mvp;

out vec4 vertex_color;

void main() {
  gl_Position = mvp * position;
  vertex_color = color;
}