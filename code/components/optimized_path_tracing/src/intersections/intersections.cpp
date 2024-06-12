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
        Vec3 normal = glm::cross(a.u, a.v);  //光源法向量
        Vec3 position = a.position;
        auto Np_dot_d = glm::dot(ray.direction, normal);
        if (Np_dot_d < 0.0000001f && Np_dot_d > -0.00000001f) return getMissRecord(); //ray与光源法向量垂直,无交点
        float dp = -glm::dot(position, normal);
        float t = (-dp - glm::dot(normal, ray.origin))/Np_dot_d; //计算光线与区域光源的交点在光线上的参数t
        if (t >= tMax || t < tMin) return getMissRecord(); 
        // cross test
        Vec3 hitPoint = ray.at(t); //计算交点位置
        Mat3x3 d{a.u, a.v, glm::cross(a.u, a.v)};
        d = glm::inverse(d);
        auto res  = d * (hitPoint - position);  //将交点的位置转换到区域光源的局部坐标系
        auto u = res.x, v = res.y;
        if ((u<=1 && u>=0) && (v<=1 && v>=0)) { //局部坐标在[0, 1]范围内，说明交点在区域光源的范围内
            return getHitRecord(t, hitPoint, normal, {});
        }
        return getMissRecord();
    }

    inline HitRecord xAABB(const Ray& ray, const SharedAABB& aabb, float tMin, float tMax){
        //对每一对平面 ,进行求交,求出tmin和tmax, 由于每对平面都是axis-aligned
        Vec3 t_in = (aabb->_min - ray.origin)/ray.direction;
        Vec3 t_out = (aabb->_max - ray.origin)/ray.direction;
        for(int i = 0; i < 3; i++){
            if(ray.direction[i]<0) std::swap(t_in[i], t_out[i]);
        }
        tMin = glm::max(glm::max(t_in.x, t_in.y), t_in.z); //tMin要取最大值
        tMax = glm::min(glm::min(t_out.x, t_out.y), t_out.z); //tMax要取最小值
        if (tMin < tMax && tMax >= 0) { //与AABB相交
            return getHitRecord(tMin, ray.at(tMin), {}, {});
            }
        return getMissRecord();
    }

    //对于叶子节点，直接求与物体的交点，还是先求AABB的交点再求物体的交点， 有不同的trade-off
    HitRecord xBVH(const Ray& ray, const SharedAABB& node, float tMin, float tMax) {
        if(node->type != AABB::Type::NOLEAF){ //叶结点, 直接与物体求交
            if(node->type == AABB::Type::SPHERE) {
                    return xSphere(ray, *node->sp, tMin, tMax);
                }
                else if(node->type == AABB::Type::TRIANGLE) {
                    return xTriangle(ray, *node->tr, tMin, tMax);
                }
                else if(node->type == AABB::Type::PLANE) {
                    return xPlane(ray, *node->pl, tMin, tMax);
                }
                else if(node->type == AABB::Type::MESH){
                    return xTriangle(ray, *node->ms, tMin, tMax);
                }
        }
        auto hitRecord = xAABB(ray, node, tMin, tMax);  //当前AABB有无交点
        if (hitRecord == nullopt) {
            return getMissRecord();     
        }
        //if (node->type == AABB::Type::NOLEAF) { // internal node
            auto left = xBVH(ray, node->left, tMin, tMax);
            auto right = xBVH(ray, node->right, tMin, tMax);
            if ((left != nullopt) && (right != nullopt)) {
                return left->t < right->t ? left : right;
            }
            else if (left != nullopt) return left;
            else if (right != nullopt) return right;
            return getMissRecord();
        //}
        // else { // leaf node
        //         if(node->type == AABB::Type::SPHERE) {
        //             return xSphere(ray, *node->sp, tMin, tMax);
        //         }
        //         else if(node->type == AABB::Type::TRIANGLE) {
        //             return xTriangle(ray, *node->tr, tMin, tMax);
        //         }
        //         else if(node->type == AABB::Type::PLANE) {
        //             return xPlane(ray, *node->pl, tMin, tMax);
        //         }
        //         else if(node->type == AABB::Type::MESH){
        //             return xTriangle(ray, *node->ms, tMin, tMax);
        //         }
        //         return getHitRecord(tMin, ray.at(tMin), {}, {});
        //     }
    }   

    int64_t getIntersectionCount() {
            return intersectCnt.load(); // 读取原子计数器的值
        }

    void resetIntersectionCount() {
        intersectCnt.store(0); // 将计数器重置为0
    }
}