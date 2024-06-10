#pragma once
#ifndef __CONDUCTOR_HPP__
#define __CONDUCTOR_HPP__

#include "Shader.hpp"

namespace OptimizedPathTracer
{
    class Conductor : public Shader
    {
    private:
        Vec3 absorbed; 
        float eta; 
        float k;
    public:
        Conductor(Material& material, vector<Texture>& textures);
        Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
    };
    
}

#endif