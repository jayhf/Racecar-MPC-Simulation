#version 330

uniform mat4 mvp;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out float height;
out vec3 normal_fs;

void main() {
  gl_Position = mvp * vec4(position.xyz, 1.0);
  height = position.y;
  normal_fs = normal;
}