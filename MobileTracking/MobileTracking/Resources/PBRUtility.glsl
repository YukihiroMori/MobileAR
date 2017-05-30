#ifndef INCLUDE_PBRUTILITY_GLSL
#define INCLUDE_PBRUTILITY_GLSL

const float Exposure = 1.0;
const float gamma = 2.2;

vec3 toLinear(vec3 v){
	return pow(v , vec3(gamma));
}

vec4 toLinear(vec4 v){
	return vec4(toLinear(v.rgb),v.a);
}

vec3 toGamma(vec3 v){
	return pow(v, vec3(1.0 / gamma));
}

vec4 toGamma(vec4 v){
	return vec4(toGamma(v.rgb), v.a);
}


#endif
