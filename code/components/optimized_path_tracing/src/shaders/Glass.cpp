#include "shaders/Glass.hpp"
#include "samplers/SamplerInstance.hpp"

namespace OptimizedPathTracer
{
     Glass::Glass(Material& material, vector<Texture>& textures)
        :  Shader           (material, textures) 
        {
            auto ior = material.getProperty<Property::Wrapper::FloatType>("ior");
            if (ior) this->ior = (*ior).value;
            else this->ior = 1.5f; //默认折射率为1.5

            auto absorbed = material.getProperty<Property::Wrapper::RGBType>("absorbed");
            if (absorbed) this->absorbed = (*absorbed).value;
            else this->absorbed = Vec3(1.f); //默认吸收率为1
        }
    
    Scattered Glass::shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const {
        //使用Schlick's approximation来计算反射率 R = R0 + (1 - R0)(1 - cosθ)^5
        //https://en.wikipedia.org/wiki/Schlick%27s_approximation
        Vec3 origin = hitPoint;
        Vec3 N = normal;
        Vec3 I = ray.direction;
        float cosTheta = glm::dot(I, N);  //入射角cos
        bool isEntering = cosTheta > 0; //是否从外部射入
        if (!isEntering) { //如果从内部射入 ,则将法向量取反,并将cosTheta取反
            N = -N; 
            cosTheta = -cosTheta;
        }

        float r0 = (1 - ior) / (1 + ior); 
        r0 = r0 * r0;   //平方
        float reflectance = r0 + (1 - r0) * std::pow((1 - cosTheta), 5); //schlick's approximation

        //Vec3 reflectionDirection = glm::reflect(I, N); //反射方向
        Vec3 reflectionDirection = glm::normalize(I - 2 * glm::dot(I, N) * N);
        Vec3 reflectRatio = reflectance * absorbed; //反射率

        //折射方向, 如果是从外部(空气)射入,则折射率为1/ior, 否则为ior
        //Vec3 refractionDirection = isEntering ? glm::refract(I, N, 1.f / ior) : glm::refract(I, N, ior); 
        Vec3 refractionDirection = isEntering ? glm::normalize(glm::refract(I, N, 1.f / ior)) : glm::normalize(glm::refract(I, N, ior));
        Vec3 refractRatio = (1 - reflectance) * absorbed; //折射率

        float sin_r = std::pow(1 - std::pow(glm::dot(I, N), 2), 0.5) / ior; //sin(折射角) = sin(入射角) / 折射率
        if (sin_r > 1.f) { //全反射
            reflectRatio = absorbed; 
            refractionDirection = Vec3(0.f);
            refractRatio = Vec3(0.f);
        }
        return {
            Ray{origin, reflectionDirection},
            reflectRatio,
            Vec3{0},
            1,
            Ray{origin, refractionDirection},
            refractRatio
        };

    }
}