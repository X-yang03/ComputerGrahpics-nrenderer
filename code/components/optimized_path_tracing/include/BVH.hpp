#pragma once
#ifndef __BVH_HPP__
#define __BVH_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"
#include "geometry/vec.hpp"
#include "shaders/ShaderCreator.hpp"
#include "AABB.hpp"
#include <algorithm>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>
namespace OptimizedPathTracer
{
    using namespace NRenderer;
    using namespace std;

    // struct BVHNode{
    //     AABB box;
    //     BVHNode* left;
    //     BVHNode* right;
    //     bool isLeaf;
    //     int start;
    //     int end;
    //     BVHNode(const AABB& box, BVHNode* left, BVHNode* right, bool isLeaf, int start, int end)
    //         : box(box), left(left), right(right), isLeaf(isLeaf), start(start), end(end)
    //     {}
    // };

    class BVHTree{

    public:
        vector<SharedAABB> aabbs;  
        SharedAABB root;
        SharedScene spscene;
        
        SharedAABB build_BVH(vector<SharedAABB>& aabbs, int start, int end){ //[start, end)
            if(start == end || start == end - 1){   //leaf node,仅有一个Entity
                return aabbs[start];
            }
            Vec3 min = Vec3(FLOAT_INF, FLOAT_INF, FLOAT_INF);
            Vec3 max = Vec3(-FLOAT_INF, -FLOAT_INF, -FLOAT_INF);
            for(int i = start; i < end; i++){
                min = Vec3(fmin(min.x, aabbs[i]->_min.x),
                            fmin(min.y, aabbs[i]->_min.y),
                            fmin(min.z, aabbs[i]->_min.z));
                max = Vec3(fmax(max.x, aabbs[i]->_max.x),
                            fmax(max.y, aabbs[i]->_max.y),
                            fmax(max.z, aabbs[i]->_max.z));
            } //计算整体的最紧包围盒
            Vec3 size = max - min; //计算包围盒的长宽高,并找到最长的轴(分布最散的轴)  
            int axis = 0;
            if(size.y > size.x) axis = 1;
            if(size.z > size.y) axis = 2;
            if(size.z > size.x) axis = 2;
            float mid = (min[axis] + max[axis]) / 2; //计算中点
            int midIndex = start;   //midIndex左边的都是小于mid的,右边的都是大于mid的
            for(int i = start; i < end; i++){
                if((aabbs[i]->_min[axis] + aabbs[i]->_max[axis]) / 2 < mid){ //在左边/上边/前边
                    swap(aabbs[i], aabbs[midIndex]);
                    midIndex++;
                }
            }
            if(midIndex == start || midIndex == end){ //全部在左边/上边/前边或者全部在右边/下边/后边
                midIndex = start + (end - start) / 2; //取中间的一个
            }
            //循环处理后,midIndex左边的都是小于mid的,右边(>=midIndex)的都是大于mid的AABB
            SharedAABB left = build_BVH(aabbs, start, midIndex);
            SharedAABB right = build_BVH(aabbs, midIndex, end);

            //return SharedAABB{new AABB(left, right)};
            return make_shared<AABB>(left, right);
        } 

        BVHTree(SharedScene spscene){
            this->spscene = spscene;

            for(auto& node: spscene->nodes){
                if(node.type == Node::Type::SPHERE){
                     aabbs.push_back(make_shared<AABB>(&(spscene->sphereBuffer[node.entity])));
                }
                else if(node.type == Node::Type::TRIANGLE){
                    aabbs.push_back(make_shared<AABB>(&(spscene->triangleBuffer[node.entity])));
                }
                else if(node.type == Node::Type::PLANE){
                    aabbs.push_back(make_shared<AABB>(&(spscene->planeBuffer[node.entity])));
                }
                // else if(node.type == Node::Type::MESH){
                //     aabbs.push_back(make_shared<AABB>(spscene->meshBuffer[node.entity]));
                // }
                else{
                    throw runtime_error("Unknown Node Type");
                }
            }
            root = build_BVH(aabbs, 0, aabbs.size());
        }     

    };
    SHARE(BVHTree);

}

#endif