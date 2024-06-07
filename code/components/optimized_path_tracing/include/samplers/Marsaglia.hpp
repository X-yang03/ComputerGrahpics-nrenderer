#pragma once
#ifndef __MARSAGLIA_HPP__
#define __MARSAGLIA_HPP__

#include "Sampler3d.hpp"
#include <ctime>

namespace OptimizedPathTracer
{
    using namespace std;
    class Marsaglia : public Sampler3d  //Marsaglia算法生成均匀分布在单位球内的随机三维向量
    {
    private:
        default_random_engine e;
        uniform_real_distribution<float> u;
    public:
        Marsaglia()
            : e               ((unsigned int)time(0) + insideSeed())
            , u               (-1, 1)
        {}

        Vec3 sample3d() override { //使用Marsaglia算法生成一个均匀分布在单位球内的随机三维向量
            float u_{0}, v_{0};
            float r2{0};
            do {
                u_ = u(e);
                v_ = u(e);
                r2 = u_*u_ + v_*v_;
            } while (r2 > 1); //如果r2大于1则重新生成,保证采样点在球内
            float x = 2 * u_ * sqrt(1 - r2);
            float y = 2 * v_ * sqrt(1 - r2);
            float z = 1 - 2 * r2;
            //x^2 + y^2 + z^2 = (2 * u_ * sqrt(1 - r2))^2 + (2 * v_ * sqrt(1 - r2))^2 + (1 - 2 * r2)^2 = 4 * r2 * (1 - r2) + 1 - 4 * r2 = 1
            return { x, y, z };
        }
    };
}

#endif