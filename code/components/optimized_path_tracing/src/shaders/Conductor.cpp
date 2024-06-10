#include "shaders/Conductor.hpp"
#include "samplers/SamplerInstance.hpp"

namespace OptimizedPathTracer
{
    Conductor::Conductor(Material& material, vector<Texture>& textures)
    :  Shader           (material, textures) 
    {
         auto absorbed = material.getProperty<Property::Wrapper::RGBType>("absorbed");
        if (absorbed) this->absorbed = (*absorbed).value;
        else this->absorbed = Vec3(1.f); //默认吸收率为1

        auto eta = material.getProperty<Property::Wrapper::FloatType>("eta");
        if (eta) this->eta = (*eta).value;
        else this->eta = 0.29;

        auto k = material.getProperty<Property::Wrapper::FloatType>("k");
        if (k) this->k = (*k).value;
        else this->k = 3.88;
    }


    Scattered Conductor::shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const {
        Vec3 origin = hitPoint;
        Vec3 N = glm::normalize(normal); //法向量
        Vec3 I = glm::normalize(ray.direction);
        float cosTheta = glm::dot(-I, N);  //入射角cos
        bool isEntering = cosTheta > 0; //是否从外部射入
        if (!isEntering) { //如果从内部射入 ,则将法向量取反,并将cosTheta取反
            N = -N; 
            cosTheta = glm::dot(-I, N);
        }

        Vec3 reflectionDirection = glm::reflect(I, N); //反射方向

        float k2 = k * k;
        Vec3 etaMinusOne = eta -  Vec3(1.f);
        Vec3 etaPlusOne = eta +  Vec3(1.f);

        Vec3 r0 = (etaMinusOne * etaMinusOne + k2) / (etaPlusOne * etaPlusOne + k2);

        Vec3 reflectance = r0 + ( Vec3(1.f) - r0) * pow(1.0f - cosTheta, 5.0f);

        Vec3 reflectRatio = isEntering ? reflectance * absorbed : Vec3(0.f); 

        return {
            Ray{origin, reflectionDirection}, //反射光线
            reflectRatio, //反射率
            Vec3{0}, 
            1
        };
    }
}