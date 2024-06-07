#include "shaders/Phong.hpp"

namespace RayCast
{
    Vec3 reflect(const Vec3& normal, const Vec3& dir) {
        return dir - 2*glm::dot(dir, normal)*normal; // 反射光线 = 入射光线 - 2 * (入射光线与法线的夹角余弦值) * 法线
    }
    Phong::Phong(Material& material, vector<Texture>& textures)
        : Shader                (material, textures)
    {
        using PW = Property::Wrapper;
        auto optDiffuseColor = material.getProperty<PW::RGBType>("diffuseColor");
        if (optDiffuseColor) diffuseColor = (*optDiffuseColor).value;
        else diffuseColor = {1, 1, 1};
        
        auto optSpecularColor = material.getProperty<PW::RGBType>("specularColor");
        if (optSpecularColor) specularColor = (*optSpecularColor).value;
        else specularColor = {1, 1, 1};

        auto optSpecularEx = material.getProperty<PW::FloatType>("specularEx");
        if (optSpecularEx) specularEx = (*optSpecularEx).value;
        else specularEx = 1;

    }
    RGB Phong::shade(const Vec3& in, const Vec3& out, const Vec3& normal) const {
        Vec3 v = in;
        Vec3 r = reflect(normal, out);
        auto diffuse = diffuseColor * glm::dot(out, normal);
        auto specular = specularColor * fabs(glm::pow(glm::dot(v, r), specularEx)); //镜面反射系数 * (R*V)^n
        return diffuse + specular;
    }
}