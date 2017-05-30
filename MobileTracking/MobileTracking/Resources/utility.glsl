#ifndef INCLUDE_UTILITY_GLSL
#define INCLUDE_UTILITY_GLSL

#define PI 3.1415926

uniform uint u_m;
uniform float u_binaryFractionFactor;

float saturate(float value){
	return clamp(value, 0.0, 1.0);
}

vec2 panorama(vec3 ray){
	return vec2(0.5 + 0.5*atan(ray.x, -ray.z)/PI, 1.0 - acos(ray.y)/PI);
}

 float radicalInverse_VdC(uint bits) {
     bits = (bits << 16u) | (bits >> 16u);
     bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
     bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
     bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
     bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
     return float(bits) * 2.3283064365386963e-10; // / 0x100000000
 }

 vec2 hammersley(uint i, uint N) {
     return vec2(float(i)/float(N), radicalInverse_VdC(i));
 }


#endif
