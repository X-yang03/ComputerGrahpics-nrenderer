#pragma once
#ifndef __PLASTIC_HPP__
#define __PLASTIC_HPP__

#include "Shader.hpp"

namespace OptimizedPathTracer
{
    class Plastic : public Shader
    {
    private:
        Vec3 albedo; //漫反射
        Vec3 specularColor; //镜面反射
        float ior; //折射率
    public:
        Plastic(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
    };
}

#endif