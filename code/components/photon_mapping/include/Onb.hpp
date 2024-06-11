#pragma once
#ifndef __ONB_HPP__
#define __ONB_HPP__

#include "geometry/vec.hpp"

namespace PhotonMapper
{
    using namespace NRenderer;
    class Onb  //Orthonormal Basis"，表示正交基
    {
    private:
        Vec3 u;
        Vec3 v;
        Vec3 w;
    public:
        Onb(const Vec3& normal) {
            w = normal;  //法线
            Vec3 a = (fabs(w.x) > 0.9) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
            //如果w与x轴接近平行，那么选择y轴方向的单位向量作为a；否则选择x轴方向的单位向量作为a,这样可以确保a与w不平行。
            v = glm::normalize(glm::cross(w, a));
            u = glm::cross(w, v); //u v w是一组正交基
        }
        ~Onb() = default;

        Vec3 local(const Vec3& v) const {  //正交基的线性组合,点乘
            return v.x*this->u + v.y*this->v + v.z * this->w;
        }
    };
}


#endif