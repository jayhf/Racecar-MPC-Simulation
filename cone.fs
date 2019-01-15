#version 330

uniform vec4 coneColor;
uniform vec3 lightDirection;
uniform float stripeStart1;
uniform float stripeStart2;
uniform float stripeEnd1;
uniform float stripeEnd2;
uniform float stripeAngle;
uniform sampler2D texture;

in float height;
in vec3 normal_fs;

out vec4 outputColor;


void main() {
  float lerpVal = gl_FragCoord.y / 500.0f;

  bool aboveStripe1 = height > stripeEnd1;
  vec4 baseColor;
  if(height < stripeStart1 || (height > stripeEnd1 && height < stripeStart2) || height > stripeEnd2){
    baseColor = coneColor;
  }
  else{
      float angle = stripeAngle - atan(normal_fs.z, normal_fs.x)*(1/(2*3.141593));
      if(height < stripeEnd1){
        baseColor = texture2D(texture, vec2(angle, (stripeEnd1 - height)/(stripeEnd1 - stripeStart1)));
      }
      else{
        baseColor = texture2D(texture, vec2(angle, (stripeEnd2 - height)/(stripeEnd2 - stripeStart2)));
      }
  }

  outputColor = vec4((baseColor * (.25 + max(0, .75 * dot(lightDirection, normal_fs)))).xyz, coneColor.w);
}
