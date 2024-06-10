#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Glass.hpp"

namespace OptimizedPathTracer
{
    class ShaderCreator
    {
    public:
        ShaderCreator() = default;
        SharedShader create(Material& material, vector<Texture>& t) {
            SharedShader shader{nullptr};
            switch (material.type) //根据材质类型创建不同的shader
            {
            case Material::LAMBERTIAN :
                shader = make_shared<Lambertian>(material, t);
                break;
            
            case Material::DIELECTRIC :
                shader = make_shared<Glass>(material, t);
                break;
            default:
                shader = make_shared<Lambertian>(material, t);
                break;
            }
            return shader;
        }
    };
}

#endif