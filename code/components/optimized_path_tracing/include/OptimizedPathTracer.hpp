#pragma once
#ifndef __OPTIMIZED_PATH_TRACER_HPP__
#define __OPTIMIZED_PATH_TRACER_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"

#include "shaders/ShaderCreator.hpp"
#include "BVH.hpp"

#include <tuple>
namespace OptimizedPathTracer
{
    using namespace NRenderer;
    using namespace std;

    class OptimizedPathTracerRenderer
    {
    public:
    private:
        SharedScene spScene;
        Scene& scene;

        SharedBVHTree bvhTree = nullptr;

        unsigned int width;
        unsigned int height;
        unsigned int depth;     //最大的trace递归数目
        unsigned int samples; //采样的光线数

        using SCam = OptimizedPathTracer::Camera;
        SCam camera;

        vector<SharedShader> shaderPrograms;
    public:
        OptimizedPathTracerRenderer(SharedScene spScene)
            : spScene               (spScene)
            , scene                 (*spScene)
            , camera                (spScene->camera)
        {
            width = scene.renderOption.width;
            height = scene.renderOption.height;
            depth = scene.renderOption.depth;
            samples = scene.renderOption.samplesPerPixel;
            //this->bvhTree = make_shared<BVHTree>(spScene);
        }
        ~OptimizedPathTracerRenderer() = default;

        using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;
        RenderResult render();
        void release(const RenderResult& r);

    private:
        void renderTask(RGBA* pixels, int width, int height, int off, int step);

        RGB gamma(const RGB& rgb);
        tuple<Vec3, Vec3> sampleOnlight(const AreaLight& light);
        RGB trace(const Ray& ray, int currDepth);
        RGB OptTrace(const Ray& ray, int currDepth);  //质量最优的采样算法，16采样率下结果可媲美普通的2048采样率
        RGB ProbablityTrace(const Ray& ray, int currDepth); //质量与速度的折中算法，16采样率下结果可媲美普通的1024采样率，但速度比OptTrace快1倍
        HitRecord closestHitObject(const Ray& r);
        tuple<float, Vec3> closestHitLight(const Ray& r);
        
    };
}

#endif