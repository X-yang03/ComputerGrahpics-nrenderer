#include "shaders/Plastic.hpp"
#include "samplers/SamplerInstance.hpp"

#include "Onb.hpp"

namespace PhotonMapper
{
    Plastic::Plastic(Material& material, vector<Texture>& textures)
        : Shader                (material, textures)
    {
        auto diffuseColor = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        if (diffuseColor) albedo = (*diffuseColor).value;
        else albedo = {1, 1, 1};

        auto specularColor = material.getProperty<Property::Wrapper::RGBType>("specularColor");
        if (specularColor) this->specularColor = (*specularColor).value;
        else this->specularColor = {1, 1, 1};

        auto ior = material.getProperty<Property::Wrapper::FloatType>("refractIndex"); 
        if (ior) this->ior = (*ior).value; // Index of Refraction 折射率
        else this->ior = 1.46f;
    }
    Scattered Plastic::shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const { 
        Vec3 origin = hitPoint;
        Vec3 N = glm::normalize(normal); //法向量
        Vec3 I = glm::normalize(ray.direction);
        float cosTheta = glm::dot(-I, N);  //入射角cos
        bool isEntering = cosTheta > 0; //是否从外部射入
        if (!isEntering) { //如果从内部射入 ,则将法向量取反,并将cosTheta取反
            N = -N; 
            cosTheta = glm::dot(-I, N);
        }

        Vec3 random = defaultSamplerInstance<HemiSphere>().sample3d();
        Onb onb{normal}; //包含normal的一组正交基
        Vec3 diffuseDir = glm::normalize(onb.local(random));

        Vec3 reflectDir = glm::reflect(I, N); //镜面反射方向

        float r0 = (1 - ior) / (1 + ior); 
        r0 = r0 * r0; 

        float reflectance = r0 + (1 - r0) * std::pow(1 - cosTheta, 5); //Schlick's approximation

        Vec3 reflectionDirection = glm::normalize(glm::mix(reflectDir, diffuseDir, 0.2)); //反射方向
        Vec3 reflectRatio = reflectance * albedo; //反射率

        Vec3 refractionDirection = isEntering ? glm::refract(I, N, 1.f / ior) : glm::refract(I, N, ior); 

        Vec3 refractRatio = (1 - reflectance) * albedo; //折射率

        if (cosTheta < 0.01f) { //全反射, 没有折射
            reflectRatio = albedo;  
            refractionDirection = Vec3(0.f);
            refractRatio = Vec3(0.f);
        }
        else if(cosTheta > 0.99f) { //全透射
            reflectRatio = Vec3(0.f);
            reflectionDirection = Vec3(0.f);
            refractRatio = albedo;
        }
        return {
            Ray{origin, reflectionDirection}, //反射光线
            reflectRatio, //反射率
            Vec3{0}, 
            1,
            Ray{origin, refractionDirection}, //折射光线
            refractRatio //折射率
        };
        
    }
}