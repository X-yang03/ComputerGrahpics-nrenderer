#pragma once
#ifndef __PHOTON_MAPPING_HPP__
#define __PHOTON_MAPPING_HPP__

#include "scene/Scene.hpp"
#include "Ray.hpp"
#include "Camera.hpp"
#include "intersections/HitRecord.hpp"

#include "shaders/ShaderCreator.hpp"
#include "KDTree.hpp"

#include <tuple>
namespace PhotonMapper
{
    using namespace NRenderer;
    using namespace std;

    class PhotonMapperRenderer
    {
    private:
        SharedScene spScene;
        Scene& scene;

        unsigned int width;
        unsigned int height;
        unsigned int depth;     //最大的trace递归数目
        unsigned int samples; //采样的光线数
        unsigned int photonNum; //光子数目

        using SCam = PhotonMapper::Camera;
        SCam camera;

        vector<SharedShader> shaderPrograms;
        vector<Photon> photons;
        KDTree photonMap;
    public:
        PhotonMapperRenderer(SharedScene spScene)
            : spScene               (spScene)
            , scene                 (*spScene)
            , camera                (spScene->camera)
        {
            width = scene.renderOption.width;
            height = scene.renderOption.height;
            depth = scene.renderOption.depth;
            samples = scene.renderOption.samplesPerPixel;
            photonNum = scene.renderOption.photonNum;
            getServer().logger.log("width: " + to_string(width));
            getServer().logger.log("height: " + to_string(height));
            getServer().logger.log("depth: " + to_string(depth));
            getServer().logger.log("samples: " + to_string(samples));
            getServer().logger.log("photonNum: " + to_string(photonNum));
        }
        ~PhotonMapperRenderer() = default;

        using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;
        RenderResult render();
        void release(const RenderResult& r);

    private:
        void renderTask(RGBA* pixels, int width, int height, int off, int step);

        RGB gamma(const RGB& rgb);
        RGB trace(const Ray& ray, int currDepth);
        HitRecord closestHitObject(const Ray& r);
        tuple<float, Vec3> closestHitLight(const Ray& r);
        void generatePhotonMap();
        void tracePhoton(const Ray& ray, const RGB& power, int depth);
    };
}

#endif