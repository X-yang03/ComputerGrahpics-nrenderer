#pragma once
#ifndef __NR_SCENE_HPP__
#define __NR_SCENE_HPP__

#include "Texture.hpp"
#include "Material.hpp"
#include "Model.hpp"
#include "Light.hpp"
#include "Camera.hpp"

namespace NRenderer
{
    struct RenderOption
    {
        unsigned int width;
        unsigned int height;
        unsigned int depth;
        unsigned int samplesPerPixel;
        unsigned int photonNum;
        RenderOption()
            : width             (500)
            , height            (500)
            , depth             (40)
            , samplesPerPixel   (16)
            , photonNum         (100000)
        {}
    };

    struct Ambient
    {
        enum class Type
        {
            CONSTANT, ENVIROMENT_MAP
        };
        Type type;
        Vec3 constant = {};
        Handle environmentMap = {};
    };

    struct Scene
    {
        Camera camera;          //摄像机

        RenderOption renderOption;  //渲染选项

        Ambient ambient;        //环境光

        // buffers
        vector<Material> materials;  //材质
        vector<Texture> textures;    //纹理

        vector<Model> models;         //模型
        vector<Node> nodes;         //节点
        // object buffer
        vector<Sphere> sphereBuffer;    //球体
        vector<Triangle> triangleBuffer;    //三角形
        vector<Plane> planeBuffer;  //平面
        vector<Mesh> meshBuffer;    //网格

        vector<Light> lights;    //灯光
        // light buffer
        vector<PointLight> pointLightBuffer;//点光源
        vector<AreaLight> areaLightBuffer;  //面光源
        vector<DirectionalLight> directionalLightBuffer;    //方向光
        vector<SpotLight> spotLightBuffer;      //聚光灯
    };
    using SharedScene = shared_ptr<Scene>;
} // namespace NRenderer


#endif