#include "RayCastRenderer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"
namespace RayCast
{
    void RayCastRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }
    RGB RayCastRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }
    auto RayCastRenderer::render() -> RenderResult {
        auto width = scene.renderOption.width;
        auto height = scene.renderOption.height;
        auto pixels = new RGBA[width*height];

        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        ShaderCreator shaderCreator{};
        for (auto& mtl : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(mtl, scene.textures));
        }

        for (int i=0; i<height; i++) {  //逐像素的渲染
            for (int j=0; j < width; j++) {
                auto ray = camera.shoot(float(j)/float(width), float(i)/float(height));  //从像素投射光线
                auto color = trace(ray);
                color = clamp(color);  //将颜色值限制在0-1之间
                color = gamma(color);   //gamma校正
                pixels[(height-i-1)*width+j] = {color, 1}; //将颜色值存入像素数组,注意这里的坐标系是左下角为原点
            }
        }

        return {pixels, width, height};
    }
    
    RGB RayCastRenderer::trace(const Ray& r) {
        if (scene.pointLightBuffer.size() < 1) return {0, 0, 0}; //没有光源
        auto& l = scene.pointLightBuffer[0]; //场景中第一个光源
        auto closestHitObj = closestHit(r);
        if (closestHitObj) {
            auto& hitRec = *closestHitObj;
            auto out = glm::normalize(l.position - hitRec.hitPoint); //光线方向的单位向量
            if (glm::dot(out, hitRec.normal) < 0) {  //光线与法线夹角大于90度
                return {0, 0, 0}; //返回黑色,因为光线是从物体的内部射出的,看不到这样的光线。
            }
            auto distance = glm::length(l.position - hitRec.hitPoint); //光源到交点的距离
            auto shadowRay = Ray{hitRec.hitPoint, out}; //阴影光线
            auto shadowHit = closestHit(shadowRay);
            auto c = shaderPrograms[hitRec.material.index()]->shade(-r.direction, out, hitRec.normal);
            if ((!shadowHit) || (shadowHit && shadowHit->t > distance)) {
                return c * l.intensity;
            }
            else {
                return Vec3{0};
            }
        }
        else {
            return {0, 0, 0};
        }
    }

    HitRecord RayCastRenderer::closestHit(const Ray& r) {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        for (auto& s : scene.sphereBuffer) {    //遍历所有球体
            auto hitRecord = Intersection::xSphere(r, s, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {  //找到距离最近的交点
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& t : scene.triangleBuffer) {  //遍历所有三角形
            auto hitRecord = Intersection::xTriangle(r, t, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& p : scene.planeBuffer) {//遍历所有平面
            auto hitRecord = Intersection::xPlane(r, p, 0.01, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        return closestHit; 
    }
}