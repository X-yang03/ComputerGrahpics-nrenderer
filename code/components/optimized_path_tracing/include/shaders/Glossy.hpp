#pragma once
#ifndef __GLOSSY_HPP__
#define __GLOSSY_HPP__

#include "Shader.hpp"

namespace OptimizedPathTracer
{
    class Glossy : public Shader
    {
    private:
        Vec3 absorbed; 
        float eta; 
        float k;
    public:
        Glossy(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
    };
    
}

#endif