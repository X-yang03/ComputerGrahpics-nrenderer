#pragma once
#ifndef __INTERSECTIONS_HPP__
#define __INTERSECTIONS_HPP__

#include "HitRecord.hpp"
#include "Ray.hpp"
#include "scene/Scene.hpp"
#include "AABB.hpp"
#include "BVH.hpp"
#include <atomic>

namespace PhotonMapper
{
    namespace Intersection
    {
        //static std::atomic<int64_t> intersectCnt = 0;
        HitRecord xTriangle(const Ray& ray, const Triangle& t, float tMin = 0.f, float tMax = FLOAT_INF);
        HitRecord xSphere(const Ray& ray, const Sphere& s, float tMin = 0.f, float tMax = FLOAT_INF);
        HitRecord xPlane(const Ray& ray, const Plane& p, float tMin = 0.f, float tMax = FLOAT_INF);
        HitRecord xAreaLight(const Ray& ray, const AreaLight& a, float tMin = 0.f, float tMax = FLOAT_INF);
        inline HitRecord xAABB(const Ray& ray, const SharedAABB& aabb, float tMin = 0.f, float tMax = FLOAT_INF);
        HitRecord xBVH(const Ray& ray, const SharedAABB& node, float tMin = 0.f, float tMax = FLOAT_INF);

        //int64_t getIntersectionCount();
        //void resetIntersectionCount(); // 将计数器重置为0
    
    }
}

#endif