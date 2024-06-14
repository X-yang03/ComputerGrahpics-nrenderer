#pragma once
#ifndef __HEMI_SPHERE_HPP__
#define __HEMI_SPHERE_HPP__

#include "Sampler3d.hpp"
#include <ctime>

namespace PhotonMapper
{
    using namespace std;
    class HemiSphere : public Sampler3d 
    {
    private:
        constexpr static float C_PI = 3.14159265358979323846264338327950288f;
        
        default_random_engine e;    // random seed
        uniform_real_distribution<float> u; // random number generator, uniform distribution
    public:
        HemiSphere()
            : e               ((unsigned int)time(0) + insideSeed()) //get a random seed
            , u               (0, 1)
        {}

        Vec3 sample3d() override {
            float epsilon1 = u(e); // random number between 0 and 1
            float epsilon2 = u(e); // random number between 0 and 1
            float r = sqrt(1 - epsilon1 * epsilon1);  //随机采样的r
            float x = cos(2*C_PI*epsilon2) * r;  //半球面立体角为2pi, 2pi*epsilon2为随机采样的theta角
            float y = sin(2*C_PI*epsilon2) * r; //半球面立体角为2pi, 2pi*epsilon2为随机采样的phi角
            float z = epsilon1; //实际上使用柱坐标, 转换为直角坐标
            return { x, y, z }; //随机采样点
        }
    };
}

#endif