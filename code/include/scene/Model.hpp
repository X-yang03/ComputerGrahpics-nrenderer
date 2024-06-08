#pragma once
#ifndef __NR_MODEL_HPP__
#define __NR_MODEL_HPP__

#include <string>
#include <vector>

#include "geometry/vec.hpp"

#include "Material.hpp"
#include "common/macros.hpp"

namespace NRenderer
{
    using namespace std;

    struct Entity {
        Handle material;
    };
    SHARE(Entity);

    struct Sphere : public Entity
    {
        Vec3 direction = {0, 0, 1};     //球体方向(极轴方向)
        Vec3 position = {0, 0, 0};          //球体位置
        float radius = { 0 };       //半径
    };
    SHARE(Sphere);
    
    struct Triangle : public Entity
    {
        union {
            struct {
                Vec3 v1;
                Vec3 v2;
                Vec3 v3;
            };
            Vec3 v[3];   //三角形的三个顶点局部坐标
        };
        Vec3 normal;        //三角形法线    
        Triangle()
            : v1            ()
            , v2            ()
            , v3            ()
            , normal         (0, 0, 1)
        {}
    };
    SHARE(Triangle);

    struct Plane : public Entity
    {
        Vec3 normal = {0, 0, 1};    //平面法线  
        Vec3 position = {};         //平面位置
        Vec3 u = {};                //第一条边
        Vec3 v = {};                //第二条边
    };
    SHARE(Plane);

    struct Mesh : public Entity
    {
        vector<Vec3> normals;   //法线,用索引访问
        vector<Vec3> positions; //位置,用索引访问
        vector<Vec2> uvs;       //uv,用索引访问
        vector<Index> normalIndices;        //法向量索引
        vector<Index> positionIndices;       //位置索引
        vector<Index> uvIndices;            //uv索引

        bool hasNormal() const {
            return normals.size() != 0;
        }

        bool hasUv() const {
            return uvs.size() != 0;
        }
    };
    SHARE(Mesh);

    struct Node
    {
        enum class Type
        {
            SPHERE = 0x0,
            TRIANGLE = 0X1,
            PLANE = 0X2,
            MESH = 0X3
        };
        Type type = Type::SPHERE;       //节点类型
        Index entity;           //实体的索引  
        Index model;            //模型索引,所属的模型
    };
    SHARE(Node);

    struct Model {
        vector<Index> nodes;   //节点索引
        Vec3 translation = {0, 0, 0};   //世界坐标的位置
        Vec3 scale = {1, 1, 1};         //缩放
    };
    SHARE(Model);
}

#endif