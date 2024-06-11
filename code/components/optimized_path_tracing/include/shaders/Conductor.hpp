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
         /*
        金（Gold, Au）:
        eta: 0.17
        k: 3.11

        银（Silver, Ag）:
        eta: 0.14
        k: 4.00

        铜（Copper, Cu）:
        eta: 0.29
        k: 3.88

        铝（Aluminum, Al）:
        eta: 1.39
        k: 7.38

        铂（Platinum, Pt）:
        eta: 2.32
        k: 4.29
        */
    };
    
}

#endif