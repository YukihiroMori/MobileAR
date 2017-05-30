#version 300 es
precision mediump float;

#define REFLECTIVE_OCCLUSION 1

#include "utility.glsl"
#include "PBRUtility.glsl"


const float kshi = 60.0;
uniform float translucency;
uniform vec4 ambient;
uniform sampler2D unit[6];
uniform sampler2D shadowMap;

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 lightMatrix;

uniform uint NumSamples;

in vec2 tc;

layout (location = 0) out vec4 fc;

vec3 cosineWeightedSampling(vec2 e){
    float phi = e.y * 2.0 * PI;
    float cosTheta = sqrt(1.0 - e.x);
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    
    return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

void main(void)
{
    vec4 albedo = toLinear(texture(unit[0] , tc));
    
    vec4 fresnel = texture(unit[1] , tc);
    
    vec4 position = texture(unit[2] , tc);
    
    vec3 normal = texture(unit[3], tc).xyz;
    
    vec3 view = normalize(position.xyz);
    
    vec3 reflection = (vec4(reflect(-view, normal), 0.0) * ModelMatrix).xyz;
    
    vec2 ref_coord;
    ref_coord = panorama(reflection);
    
    //fresnel = vec4(1.0 -  max(pow(dot(normalize(view) , normalize(normal)),5.0), 0.0));
    
    vec4 diffuse = vec4(0.0);
    
#if REFLECTIVE_OCCLUSION
    vec4 specular = toLinear(vec4(0.0));
#else
    vec4 specular = mix(toLinear(ambient) , toLinear(texture(unit[5], ref_coord)), fresnel);
#endif
    
    for (uint i = 0u; i < NumSamples; ++i){
        vec4 v = ModelMatrix * vec4(cosineWeightedSampling(hammersley(i , NumSamples)) , 1.0);
        vec2 coord;
        coord = panorama(v.xyz);
        
        vec4 radiance = toLinear(texture(unit[5], coord));
        
        diffuse += max(dot(v.xyz,normal), 0.0) * radiance;
        
#if REFLECTIVE_OCCLUSION
        specular += pow(max(dot(v.xyz, reflection), 0.0), kshi) * radiance;
#endif
    }
    
    diffuse *= 4.0 / float(NumSamples);
    
#if REFLECTIVE_OCCLUSION
    specular *= (kshi + 8.0) / float(NumSamples);
#endif
    
    //fc = albedo;
    
    //fc = fresnel;
    
    //fc = diffuse;
    
    //fc = specular;
    
    //fc = vec4(vec3(position) * 0.5 + 0.5 , 1.0);
    
    //fc = vec4(normal * 0.5 + 0.5, 1.0);
    
    //fc = vec4(texture(unit[4], tc).r);
    
    //fc = albedo * diffuse;
    
    fc = albedo * mix(diffuse, ambient , translucency);
    
    fc = mix(fc, specular, fresnel);
    
    fc *= Exposure;
    
    fc = toGamma(fc);
    
    fc = mix(ambient, fc , albedo.a) * albedo.a;
    
    //fc = toLinear(texture(unit[5], ref_coord));
    
}
