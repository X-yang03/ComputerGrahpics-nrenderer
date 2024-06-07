#pragma once
#ifndef __UNIFORM_IN_SQUARE_HPP__
#define __UNIFORM_IN_SQUARE_HPP__

#include "Sampler2d.hpp"
#include <ctime>

namespace OptimizedPathTracer
{
    using namespace std;
    class UniformInSquare: public Sampler2d //UniformInSquare算法生成均匀分布在单位正方形内的随机二维向量
    {
    private:
        default_random_engine e;
        uniform_real_distribution<float> u;
    public:
        UniformInSquare()
            : e               ((unsigned int)time(0) + insideSeed())
            , u               (-1, 1)
        {}
        Vec2 sample2d() override {
            return {u(e), u(e)};
        }
    };
}

#endif