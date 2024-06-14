#pragma once
#ifndef __SCATTERED_HPP__
#define __SCATTERED_HPP__

#include "Ray.hpp"

namespace OptimizedPathTracer
{
    struct Scattered //散射
    {
        Ray ray = {};
        Vec3 attenuation = {}; //衰减
        Vec3 emitted = {};
        float pdf = {0.f};   //概率密度函数
        Ray refractionDir = {};
        Vec3 refractRatio = {};
    };
    
}

#endif