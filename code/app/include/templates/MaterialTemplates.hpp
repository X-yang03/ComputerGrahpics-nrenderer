#ifndef __NR_MATERIAL_TEMPLATES_HPP__
#define __NR_MATERIAL_TEMPLATES_HPP__

#include <unordered_map>

#include "scene/Material.hpp"
#include "common/macros.hpp"

namespace NRenderer
{
    class DLL_EXPORT MaterialTemplates
    {
    public:
        struct Template
        {
            unsigned int index;
            string key;
            vector<Property> props;
        };

        static unordered_map<int, Template> templates;

    private:
        struct InitTemplates
        {
            template<typename T>
            void PushProp(Template& p, T&& arg) {
                p.props.push_back(arg);
            }
            template<typename T, typename ...Args>
            void PushProp(Template& p, T&& arg1, Args&& ...args) {
                p.props.push_back(arg1);
                PushProp(p, args...);
            }
            template<typename ...Args>
            void Add(unordered_map<int, Template>& templates, unsigned int index, const string& name, Args&& ...args) {
                templates[index] = {
                    index, name, {}
                };
                PushProp(templates[index], args...);
            }
            InitTemplates(unordered_map<int, Template>& templates) {
                using PW = Property::Wrapper;
                Add(templates, 0, string("Lambertian"), //漫反射有两个属性, 漫反射颜色, 漫反射贴图
                    Property("diffuseColor", PW::RGBType{}),
                    Property("diffuseMap", PW::TextureIdType{})
                );
                Add(templates, 1, string("Phong"), //Phong有4个属性, 漫反射颜色, 漫反射贴图, 镜面反射颜色, 镜面反射指数
                    Property("diffuseColor", PW::RGBType{}),
                    Property("diffuseMap", PW::TextureIdType{}),
                    Property("specularColor", PW::RGBType{}),
                    Property("specularEx", PW::FloatType{})
                );
                Add(templates, 2, string("Dielectric"), //折射材质有2个属性, 折射率, 吸收率
                    Property("ior", PW::FloatType{float(1.5)}),
                    Property("absorbed", PW::RGBType{RGB{1, 1, 1}})
                );
                Add(templates, 3, string("Conductor"), //导体有3个属性, 自身颜色(吸收率),实部折射率eta和消光系数k
                    Property("absorbed", PW::RGBType{RGB{1, 1, 1}}),
                    Property("eta", PW::FloatType{float(0.29)}),
                    Property("k", PW::FloatType{float(3.88)})  //copper
                );
                Add(templates, 4, string("Plastic"), //塑料有3个属性, 漫反射颜色, 镜面反射颜色, 镜面反射指数
                    Property("diffuseColor", PW::RGBType{}),
                    Property("specularColor", PW::RGBType{}),
                    Property("refractIndex", PW::FloatType{})
                );
                Add(templates, 5, string("Glossy"), 
                    Property("absorbed", PW::RGBType{RGB{1, 1, 1}}),
                    Property("eta", PW::FloatType{float(0.29)}),
                    Property("k", PW::FloatType{float(3.88)})  //copper
                );
                
            }
        };
        static InitTemplates initTemplates;
    };
}

#endif