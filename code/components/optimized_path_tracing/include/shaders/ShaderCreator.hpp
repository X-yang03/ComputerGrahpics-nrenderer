#pragma once
#ifndef __SHADER_CREATOR_HPP__
#define __SHADER_CREATOR_HPP__

#include "Shader.hpp"
#include "Lambertian.hpp"
#include "Glass.hpp"
#include "Conductor.hpp"
#include "Plastic.hpp"
#include "Glossy.hpp"
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
            case Material::CONDUCTOR :
                shader = make_shared<Conductor>(material, t);
                break;

            case Material::PLASTIC :
                shader = make_shared<Plastic>(material, t);
                break;

            case Material::GLOSSY :
                shader = make_shared<Glossy>(material, t);
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