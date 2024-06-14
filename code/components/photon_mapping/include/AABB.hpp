#pragma once
#ifndef __AABB_HPP__
#define __AABB_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"
#include "geometry/vec.hpp"
#include "shaders/ShaderCreator.hpp"
namespace PhotonMapper
{
    using namespace NRenderer;
    using namespace std;
    
    class AABB{
    public:
        Vec3 _min;  //左下角
        Vec3 _max;   //右上角
        enum class Type
        {
            NOLEAF = 0x0,
            SPHERE = 0x1,
            TRIANGLE = 0X2,
            PLANE = 0X3,
            MESH = 0X4
        };
        Type type = Type::NOLEAF;

        //SharedEntity entity = nullptr;
        SHARE(AABB);
        SharedAABB left = nullptr;
        SharedAABB right = nullptr;

        union
        {
            Sphere* sp;
            Triangle* tr;
            Plane* pl;
            Triangle* ms;
        };

        AABB() = default;

        AABB(SharedAABB left, SharedAABB right){
            _min = Vec3(fmin(left->_min.x, right->_min.x),
                        fmin(left->_min.y, right->_min.y),
                        fmin(left->_min.z, right->_min.z));
            _max = Vec3(fmax(left->_max.x, right->_max.x),
                        fmax(left->_max.y, right->_max.y),
                        fmax(left->_max.z, right->_max.z));
            this->left = left;
            this->right = right;
            this->type = Type::NOLEAF;
        }

        AABB(Sphere* sp){
            type = Type::SPHERE;
            this->sp = sp;
            float r = sp->radius;
            Vec3 pos = sp->position;
            _min = pos - Vec3(r, r, r);
            _max = pos + Vec3(r, r, r);
        }

        AABB(Triangle* tr){
            type = Type::TRIANGLE;
            this->tr = tr;
            Vec3 v1 = tr->v1;
            Vec3 v2 = tr->v2;
            Vec3 v3 = tr->v3;
            _min = Vec3(fmin(v1.x, fmin(v2.x, v3.x)),
                        fmin(v1.y, fmin(v2.y, v3.y)),
                        fmin(v1.z, fmin(v2.z, v3.z)));
            _max = Vec3(fmax(v1.x, fmax(v2.x, v3.x)),
                        fmax(v1.y, fmax(v2.y, v3.y)),
                        fmax(v1.z, fmax(v2.z, v3.z)));
            if(_min.x == _max.x)
                _max.x += 0.1f;
            else if(_min.y == _max.y)
                _max.y += 0.1f;
            else if(_min.z == _max.z)
                _max.z += 0.1f;  //如果是平面的话,加上一个微小的偏移，形成包围盒
        }   

        AABB(Plane* pl){   //Todo: could be optimized later
            type = Type::PLANE;
            this->pl = pl;
            Vec3 n = pl->normal;
            Vec3 p = pl->position;
            float epsilon = 0.1f;
            Vec3 p2 = p + pl->u;
            Vec3 p3 = p + pl->v;    
            Vec3 p4 = p + pl->u + pl->v;

            p -= epsilon * n;
            p2 -= epsilon * n;
            p3 += epsilon * n;
            p4 += epsilon * n;

            _min = Vec3(fmin(p.x, fmin(p2.x, fmin(p3.x, p4.x))),
                        fmin(p.y, fmin(p2.y, fmin(p3.y, p4.y))),
                        fmin(p.z, fmin(p2.z, fmin(p3.z, p4.z))));

            _max = Vec3(fmax(p.x, fmax(p2.x, fmax(p3.x, p4.x))),
                        fmax(p.y, fmax(p2.y, fmax(p3.y, p4.y))),
                        fmax(p.z, fmax(p2.z, fmax(p3.z, p4.z))));
        }

        AABB(Mesh* ms, int index){   //mesh相当于带有material的triangle
            type = Type::MESH;
            Vec3 min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
            Vec3 max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            Vec3 v1 = ms->positions[ms->positionIndices[index]];
            Vec3 v2 = ms->positions[ms->positionIndices[index + 1]];
            Vec3 v3 = ms->positions[ms->positionIndices[index + 2]];   //该mesh的三个顶点

            this->ms = new Triangle();
            this->ms->v1 = v1;
            this->ms->v2 = v2;
            this->ms->v3 = v3;  //保存为三角形
            this->ms->material = ms->material;
            this->ms->normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            min = Vec3(fmin(v1.x, fmin(v2.x, v3.x)),
                        fmin(v1.y, fmin(v2.y, v3.y)),
                        fmin(v1.z, fmin(v2.z, v3.z)));
            max = Vec3(fmax(v1.x, fmax(v2.x, v3.x)),
                        fmax(v1.y, fmax(v2.y, v3.y)),
                        fmax(v1.z, fmax(v2.z, v3.z)));

            _min = min;
            _max = max;
            if(_min.x == _max.x)
                _max.x += 0.1f;
            else if(_min.y == _max.y)
                _max.y += 0.1f;
            else if(_min.z == _max.z)
                _max.z += 0.1f;  //如果是平面的话,加上一个微小的偏移，形成包围盒
        }


    };
    SHARE(AABB);
}


#endif