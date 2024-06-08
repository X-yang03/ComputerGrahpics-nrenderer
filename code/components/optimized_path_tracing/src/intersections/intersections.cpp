#include "intersections/intersections.hpp"
#include "server/Server.hpp"
namespace OptimizedPathTracer::Intersection
{
    HitRecord xTriangle(const Ray& ray, const Triangle& t, float tMin, float tMax) {
        intersectCnt++;
        const auto& v1 = t.v1;
        const auto& v2 = t.v2;
        const auto& v3 = t.v3;
        const auto& normal = t.normal;
        auto e1 = v2 - v1;
        auto e2 = v3 - v1;
        auto P = glm::cross(ray.direction, e2);
        float det = glm::dot(e1, P);
        Vec3 T;
        if (det > 0) T = ray.origin - v1;
        else { T = v1 - ray.origin; det = -det; }
        if (det < 0.000001f) return getMissRecord();
        float u, v, w;
        u = glm::dot(T, P);
        if (u > det || u < 0.f) return getMissRecord();
        Vec3 Q = glm::cross(T, e1);
        v = glm::dot(ray.direction, Q);
        if (v < 0.f || v + u > det) return getMissRecord();
        w = glm::dot(e2, Q);
        float invDet = 1.f / det;
        w *= invDet;
        if (w >= tMax || w < tMin) return getMissRecord();
        return getHitRecord(w, ray.at(w), normal, t.material);

    }
    HitRecord xSphere(const Ray& ray, const Sphere& s, float tMin, float tMax) {
         intersectCnt++;
        const auto& position = s.position;
        const auto& r = s.radius;
        Vec3 oc = ray.origin - position;
        float a = glm::dot(ray.direction, ray.direction);
        float b = glm::dot(oc, ray.direction);
        float c = glm::dot(oc, oc) - r*r;
        float discriminant = b*b - a*c;
        float sqrtDiscriminant = sqrt(discriminant);
        if (discriminant > 0) {
            float temp = (-b - sqrtDiscriminant) / a;
            if (temp < tMax && temp >= tMin) {
                auto hitPoint = ray.at(temp);
                auto normal = (hitPoint - position)/r;
                return getHitRecord(temp, hitPoint, normal, s.material);
            }
            temp = (-b + sqrtDiscriminant) / a;
            if (temp < tMax && temp >= tMin) {
                auto hitPoint = ray.at(temp);
                auto normal = (hitPoint - position)/r;
                return getHitRecord(temp, hitPoint, normal, s.material);
            }
        }
        return getMissRecord();
    }
    HitRecord xPlane(const Ray& ray, const Plane& p, float tMin, float tMax) {
         intersectCnt++;
        auto Np_dot_d = glm::dot(ray.direction, p.normal);
        if (Np_dot_d < 0.0000001f && Np_dot_d > -0.00000001f) return getMissRecord();
        float dp = -glm::dot(p.position, p.normal);
        float t = (-dp - glm::dot(p.normal, ray.origin))/Np_dot_d;
        if (t >= tMax || t < tMin) return getMissRecord();
        // cross test
        Vec3 hitPoint = ray.at(t);
        Vec3 normal = p.normal;
        Mat3x3 d{p.u, p.v, glm::cross(p.u, p.v)};
        d = glm::inverse(d);
        auto res  = d * (hitPoint - p.position);
        auto u = res.x, v = res.y;
        if ((u<=1 && u>=0) && (v<=1 && v>=0)) {
            return getHitRecord(t, hitPoint, normal, p.material);
        }
        return getMissRecord();
    }
    HitRecord xAreaLight(const Ray& ray, const AreaLight& a, float tMin, float tMax) {
        Vec3 normal = glm::cross(a.u, a.v);
        Vec3 position = a.position;
        auto Np_dot_d = glm::dot(ray.direction, normal);
        if (Np_dot_d < 0.0000001f && Np_dot_d > -0.00000001f) return getMissRecord();
        float dp = -glm::dot(position, normal);
        float t = (-dp - glm::dot(normal, ray.origin))/Np_dot_d;
        if (t >= tMax || t < tMin) return getMissRecord();
        // cross test
        Vec3 hitPoint = ray.at(t);
        Mat3x3 d{a.u, a.v, glm::cross(a.u, a.v)};
        d = glm::inverse(d);
        auto res  = d * (hitPoint - position);
        auto u = res.x, v = res.y;
        if ((u<=1 && u>=0) && (v<=1 && v>=0)) {
            return getHitRecord(t, hitPoint, normal, {});
        }
        return getMissRecord();
    }

    HitRecord xAABB(const Ray& ray, const SharedAABB& aabb, float tMin, float tMax){
         //intersectCnt++;
        for (int i = 0; i < 3; i++) {  
            //对每一对平面 ,进行求交,求出tmin和tmax, 由于每对平面都是axis-aligned
            //所以只用考虑在这对平面法向量分量上的tmin和tmax
            float invD = 1.0f / ray.direction[i];
            float t_in = (aabb->_min[i] - ray.origin[i]) * invD; //分量相除 得到tmin
            float t_out = (aabb->_max[i] - ray.origin[i]) * invD; //分量相除 得到tmax
            if (invD < 0.0f)  std::swap(t_in, t_out);
            tMin = t_in > tMin ? t_in : tMin; //tMin要取最大值
            tMax = t_out < tMax ? t_out : tMax; //tMax要取最小值
            if (tMin > tMax) return getMissRecord(); //无交点
        }
        if (tMin < tMax && tMax >= 0) { //与AABB相交
            return getHitRecord(tMin, ray.at(tMin), {}, {});
            }
        return getMissRecord();
    }

    HitRecord xBVH(const Ray& ray, const SharedAABB& node, float tMin, float tMax) {

        auto hitRecord = xAABB(ray, node, tMin, tMax);  //当前AABB有无交点
        if (hitRecord == nullopt) {
            //getServer().logger.log("here");
            return getMissRecord();     
        }
        if (node->left  && node->right) { // internal node
            auto left = xBVH(ray, node->left, tMin, tMax);
            auto right = xBVH(ray, node->right, tMin, tMax);
            if ((left != nullopt) && (right != nullopt)) {
                return left->t < right->t ? left : right;
            }
            else if (left != nullopt) return left;
            else if (right != nullopt) return right;
            return getMissRecord();
        }
        else { // leaf node
                if(node->type == AABB::Type::SPHERE) {
                    return xSphere(ray, *node->sp, tMin, tMax);
                }
                else if(node->type == AABB::Type::TRIANGLE) {
                    return xTriangle(ray, *node->tr, tMin, tMax);
                }
                else if(node->type == AABB::Type::PLANE) {
                    return xPlane(ray, *node->pl, tMin, tMax);
                }
                return getHitRecord(tMin, ray.at(tMin), {}, {});
            }
    }   

    int64_t getIntersectionCount() {
            return intersectCnt.load(); // 读取原子计数器的值
        }
}