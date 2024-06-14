#include "shaders/Lambertian.hpp"
#include "samplers/SamplerInstance.hpp"

#include "Onb.hpp"

namespace OptimizedPathTracer
{
    Lambertian::Lambertian(Material& material, vector<Texture>& textures)
        : Shader                (material, textures)
    {
        auto diffuseColor = material.getProperty<Property::Wrapper::RGBType>("diffuseColor");
        if (diffuseColor) albedo = (*diffuseColor).value;
        else albedo = {1, 1, 1};
    }
    Scattered Lambertian::shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const { 
        Vec3 origin = hitPoint;
        Vec3 random = defaultSamplerInstance<HemiSphere>().sample3d();

        Onb onb{normal}; //包含normal的一组正交基
        Vec3 direction = glm::normalize(onb.local(random)); //将随机采样的方向, 与onb线性组合,转换到世界坐标系

        float pdf = 1/(2*PI);  //蒙特卡洛随机采样, 需要除以一个半球的面积,即2pi

        auto attenuation = albedo / PI; 

        return { //返回散射光线，衰减，自发光，pdf
            Ray{origin, direction},
            attenuation,
            Vec3{0},
            pdf
        };
    }
}