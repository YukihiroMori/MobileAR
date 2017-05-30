#version 300 es 
precision mediump float;

#include "utility.glsl"
#include "PBRUtility.glsl"

#define OCCLUSION 1

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

uniform sampler2D unit[6];

uniform float roughness;
uniform float f0;

uniform uint NumSamples;

uniform vec4 ambient;

in vec2 tc;

out vec4 fc;

void calculateBasis(out vec3 TangentX,out vec3 TangentY,in vec3 N){
    vec3 UpVector = abs(N.x) > 0.00001 ? vec3(0,1,0) : vec3(1,0,0);
    TangentX = normalize( cross( UpVector , N ) );
    TangentY = cross( N, TangentX );
}

vec3 cosineWeightedSampling(vec2 e){
    float phi = e.y * 2.0 * PI;
    float cosTheta = sqrt(1.0 - e.x);
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    
    return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

vec3 brdfLambert(vec2 randomPoint, mat3 basis, vec3 N){
    vec3 LtangentSpace = cosineWeightedSampling(randomPoint);
    
    vec3 L = basis * LtangentSpace;
    
#if OCCLUSION
    vec4 q = ProjectionMatrix * vec4(L , 1.0);
    q = q * 0.5 / q.w + 0.5;
    float visibility = step(texture(unit[4], q.xy).x , q.z);
#else
    float visibility = 1.0;
#endif
    
    vec4 lamb = texture(unit[0] , tc) * saturate(dot(N,L)) * textureLod(unit[5] , panorama(L) , 5.0) * visibility;
    
    return lamb.rgb;
}

vec3 uniformSampling(vec2 e){
    float phi = e.y * 2.0 * PI;
    float cosTheta = 1.0 - e.x;
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    
    return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

vec3 microfacetWeightedSampling(vec2 e , float r4){
    
    float phi = 2.0 * PI * e.y;
    float cosTheta = sqrt((1.0 - e.x) / (1.0 + (r4 - 1.0) * e.x));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    
    float x = sinTheta * cos(phi);
    float y = sinTheta * sin(phi);
    float z = cosTheta;
    
    return vec3(x, y, z);
}

float fresnelSchlick(float VdotH){
    return f0 + (1.0 - f0) * pow(1.0 - VdotH, 5.0);
}

float geometricShadowingSchlickBeckmann(float NdotV, float k){
    return NdotV / (NdotV * (1.0 - k) + k);
}

float geometricShadowingSmith(float NdotL, float NdotV, float k){
    return geometricShadowingSchlickBeckmann(NdotL, k) * geometricShadowingSchlickBeckmann(NdotV, k);
}

vec3 brdfCookTorrance(vec2 randomPoint, mat3 basis, vec3 N, vec3 V, float r4 , float k){
    vec3 noColor = vec3(0.0, 0.0, 0.0);
    
    vec3 HtangentSpace = microfacetWeightedSampling(randomPoint , r4);
    
    vec3 H = basis * HtangentSpace;
    
    vec3 L = reflect( -V, H );
    
    float NdotL = dot(N, L);
    float NdotV = dot(N, V);
    float NdotH = dot(N, H);
    
    if (NdotL > 0.0 && NdotV > 0.0){
        float VdotH = dot(V, H);
        
        float F = fresnelSchlick(VdotH);
        
        float G = geometricShadowingSmith(NdotL, NdotV, k);
        
        float colorFactor = F * G * VdotH / (NdotV * NdotH);
        
        if(isnan(colorFactor)){
            return noColor;
        }
        
#if OCCLUSION
        vec4 q = ProjectionMatrix * vec4(L , 1.0);
        q = q * 0.5 / q.w + 0.5;
        float visibility = step(texture(unit[4], q.xy).x , q.z);
#else
        float visibility = 1.0;
#endif
        
        return textureLod(unit[5], panorama(L) , 3.0).rgb * colorFactor * saturate(dot(N , L));
    }
    
    return noColor;
}


float CalcLinearZ(float depth) {
    const float zFar = 3000.0;
    const float zNear = 1.0;
    
    float linear = zNear / (zFar - depth * (zFar - zNear)) * zFar;
    
    return linear;
}

vec3 WorldPosFromDepth(float depth) {
    float z = depth * 2.0 - 1.0;
    
    vec4 clipSpacePosition = vec4(tc * 2.0 - 1.0, z, 1.0);
    vec4 viewSpacePosition = inverse(ProjectionMatrix) * clipSpacePosition;
    
    vec4 worldSpacePosition = inverse(ViewMatrix) * viewSpacePosition;
    
    worldSpacePosition /= worldSpacePosition.w;
    
    return worldSpacePosition.xyz;
}

void main(void){
    vec4 albedo = toLinear(texture(unit[0] , tc));
    vec4 fresnel = texture(unit[1] , tc);
    vec4 position = texture(unit[2] , tc);
    vec3 normal = texture(unit[3] , tc).xyz;
    float depth = texture(unit[4], tc).x;
    vec3 eye = -normalize(position.xyz/100.0);
    
    vec4 pos = vec4(WorldPosFromDepth(depth) , 1.0);
    
    vec3 TangentX;
    vec3 TangentY;
    
    calculateBasis(TangentX,TangentY,normal);
    
    vec3 color = vec3(0.0, 0.0, 0.0);
    
    mat3 basis = mat3(TangentX, TangentY, normal);
    
    vec2 randomPoint;
    
    float r4 = roughness * roughness * roughness * roughness;
    
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    
    for(uint sampleIndex = 0u; sampleIndex < NumSamples ; sampleIndex++){
        randomPoint = hammersley(sampleIndex , NumSamples);
        
        color += brdfLambert(randomPoint, basis, normal);
        
        color += brdfCookTorrance(randomPoint, basis, normal, eye, r4, k);
        
        //color += mix(brdfLambert(randomPoint, basis, normal), brdfCookTorrance(randomPoint, basis, normal, eye, r4, k), fresnel.xyz); 
    }
    
    vec4 diffuse = vec4(color * 4.0 / float(NumSamples) , 1.0);
    
    fc = mix(albedo , diffuse , fresnel.r);
    
    fc *= Exposure;
    
    fc = mix(ambient * albedo.a, fc , albedo.a);
    
}
