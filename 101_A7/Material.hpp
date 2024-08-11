//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { 
    DIFFUSE,
    MICROFACET,
    SPECULAR,
};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks; // diffuse and specular reflection coefficient
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

        // GGX, this function calculate the shadow masking in one direction
    static inline float geometrySchlickGGX(float NdotV, float roughness) {
        float r = roughness + 1.0f;
        float k = (r * r) / 8.0f;
        return NdotV / (NdotV * (1.0f - k) + k);
    }

    // GGX, this function calculate the shadow masking in two directions, incident and outgoing
    static inline float geometrySmith(const Vector3f& N, const Vector3f& V, const Vector3f& L, float roughness) {
        float NdotV = std::max(dotProduct(N, V), 0.0f);
        float NdotL = std::max(dotProduct(N, L), 0.0f);
        float ggx2 = Material::geometrySchlickGGX(NdotV, roughness);
        float ggx1 = Material::geometrySchlickGGX(NdotL, roughness);
        return ggx1 * ggx2;
    }

    // GGX, this function calculate the distribution of the microfacets, H is the half vector of L and V
    static inline float distributionGGX(const Vector3f& N, const Vector3f& H, float roughness) {
        float a = roughness * roughness;
        float NdotH = std::max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH * NdotH;

        float denom = (NdotH2 * (a - 1.0f) + 1.0f);
        denom = M_PI * denom * denom;
        return a / denom;
    }

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        case MICROFACET:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);

            break;
        }
        case SPECULAR:
        { // reflect around the normal and add some noise
            Vector3f reflected = reflect(wi, N);
            Vector3f noise(get_random_float() * 0.1f, get_random_float() * 0.1f, get_random_float() * 0.1f);
            reflected = normalize(reflected + noise);
            return toWorld(reflected, N); // Return the final reflected direction
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        case MICROFACET:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case SPECULAR:
        {
            // perfect reflection
            return 0.98f;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:{
            // calculate the contribution of diffuse model
            float cosalpha = dotProduct(N, -wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI; // constant, Kd is the diffuse reflection coefficient
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case SPECULAR:
        case MICROFACET:{ // remember that wo is the negative of the outgoing direction
            
            // calculate the contribution of diffuse model

            float kr; // fresnel term
            fresnel(wi, N, ior, kr); //ior is the refractive index of the material
            // shadow masking term, if the angle is too large, the shadow masking term is 0
            float roughness = 0.0f;
            if(m_type = MICROFACET)
                roughness = std::sqrt(2.0f / (specularExponent + 2.0f)); // roughness of the material
            float G = Material::geometrySmith(N, wi, -wo, roughness);
            // normal distribution, imagine specularExponent make the highlight circle, which is the solid angle affected by one pointlight
            float D = Material::distributionGGX(N, (wi - wo).normalized(), roughness);
            
            // std::cout<<D<<"\t"<<G<<"\t"<<kr<<std::endl;

            return Kd * (D * G * kr 
            / (4 * std::max(dotProduct(N, wi),0.001f) * std::max(dotProduct(N, -wo),0.001f)));
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
