#include "server/Server.hpp"

#include "OptimizedPathTracer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"

namespace OptimizedPathTracer
{
    RGB OptimizedPathTracerRenderer::gamma(const RGB& rgb) {
        return glm::sqrt(rgb);
    }

    void OptimizedPathTracerRenderer::renderTask(RGBA* pixels, int width, int height, int off, int step) {
        for(int i=off; i<height; i+=step) {
            for (int j=0; j<width; j++) {
                Vec3 color{0, 0, 0};
                for (int k=0; k < samples; k++) {
                    auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                    float rx = r.x;
                    float ry = r.y;
                    float x = (float(j)+rx)/float(width);
                    float y = (float(i)+ry)/float(height); //随机采样的光线方向
                    auto ray = camera.shoot(x, y); //打出光线
                    color += trace(ray, 0); //路径追踪渲染
                }
                color /= samples; //平均
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }
    }

    auto OptimizedPathTracerRenderer::render() -> RenderResult {
        // shaders
        shaderPrograms.clear();
        ShaderCreator shaderCreator{};
        for (auto& m : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
        }

        RGBA* pixels = new RGBA[width*height]{};

        // 局部坐标转换成世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);

        this->bvhTree = make_shared<BVHTree>(spScene);
        bvhTree->printTree(bvhTree->root, 0);

        int nodeNum = bvhTree->aabbs.size();
        cout<<"nodeNum: "<<nodeNum<<endl;

        const auto taskNums = 16;
        thread t[taskNums];
        for (int i=0; i < taskNums; i++) {
            t[i] = thread(&OptimizedPathTracerRenderer::renderTask,
                this, pixels, width, height, i, taskNums); //多线程渲染
        }
        for(int i=0; i < taskNums; i++) {
            t[i].join();
        }
        getServer().logger.log("Done...");

        int64_t totalIntersections = Intersection::getIntersectionCount();
        cout << "Total intersection calls: " << totalIntersections << std::endl;
        Intersection::resetIntersectionCount();


        return {pixels, width, height};
    }

    void OptimizedPathTracerRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    HitRecord OptimizedPathTracerRenderer::closestHitObject(const Ray& r) {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        float closest1 = FLOAT_INF;
        auto hitRecord1 = Intersection::xBVH(r, bvhTree->root, 0.000001, closest); //BVH加速
        if (hitRecord1 && hitRecord1->t < closest1 ) {
            closest1 = hitRecord1->t;
            return hitRecord1;
        }
        // int64_t totalIntersections = Intersection::getIntersectionCount();
        // Intersection::resetIntersectionCount();
        // assert(totalIntersections < 16);
        // optNum+=totalIntersections;

        //return hitRecord1;

        // for (auto& s : scene.sphereBuffer) {
        //     auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
        //     if (hitRecord && hitRecord->t < closest) {
        //         closest = hitRecord->t;
        //         closestHit = hitRecord;
        //     }
        // }
        // for (auto& t : scene.triangleBuffer) {
        //     auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
        //     if (hitRecord && hitRecord->t < closest) {
        //         closest = hitRecord->t;
        //         closestHit = hitRecord;
        //     }
        // }
        // for (auto& p : scene.planeBuffer) {
        //     auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
        //     if (hitRecord && hitRecord->t < closest) {
        //         closest = hitRecord->t;
        //         closestHit = hitRecord;
        //     }
        // }
        // totalIntersections = Intersection::getIntersectionCount();
        // Intersection::resetIntersectionCount();
        // //assert(totalIntersections == 16);
        // if(totalIntersections != 16){
        //     cout<<"totalIntersections: "<<totalIntersections<<endl;
        // }
        // ordNum+=totalIntersections;
        
        return closestHit; 
    }
    
    tuple<float, Vec3> OptimizedPathTracerRenderer::closestHitLight(const Ray& r) {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});  //使用INF初始化,表示最近交点在无穷远处
        //Cornell Box中只有一个面光源
        for (auto& a : scene.areaLightBuffer) {
            auto hitRecord = Intersection::xAreaLight(r, a, 0.000001, closest->t); //计算光线r与区域光源a的交点，得到一个HitRecord类型的对象hitRecord
            if (hitRecord && closest->t > hitRecord->t) { //ray r 和区域光有交点
                closest = hitRecord;
                v = a.radiance; //radianc相当于发出的光线
            }
        }
        //迭代之后,找到光线r与所有面光源的最近交点
        return { closest->t, v };
    }

    RGB OptimizedPathTracerRenderer::trace(const Ray& r, int currDepth) {
        if (currDepth == depth) return scene.ambient.constant; //递归数目达到depth
        auto hitObject = closestHitObject(r);   //光线最近的射中物体
        auto [ t, emitted ] = closestHitLight(r);   
        // hit object
        if (hitObject && hitObject->t < t) { //hitObject在相机和面光源之间
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            if(spScene->materials[mtlHandle.index()].type == Material::LAMBERTIAN){
                auto scatteredRay = scattered.ray;
                auto attenuation = scattered.attenuation;
                auto emitted = scattered.emitted;
                auto next = trace(scatteredRay, currDepth+1);
                float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
                float pdf = scattered.pdf;
                /**
                 * emitted      - Le(p, w_0)
                 * next         - Li(p, w_i)
                 * n_dot_in     - cos<n, w_i>
                 * atteunation  - BRDF
                 * pdf          - p(w)
                 **/
                return emitted + attenuation * next * n_dot_in / pdf; //自发光emitted, 加上来自外部的光线的亮度
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::DIELECTRIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :trace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :trace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::CONDUCTOR){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :trace(reflect, currDepth + 1); 
                return reflectRGB * reflectRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::PLASTIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :trace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :trace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::GLOSSY){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :trace(reflect, currDepth + 1); 
                return reflectRGB * reflectRatio;
            }
            else{
                return Vec3(0.f);
            }
        }
        // 
        else if (t != FLOAT_INF) {  //hitObject在面光源,直接返回面光源的激发亮度
            return emitted;
        }
        else {
            return Vec3{0}; //没有hitObject,也没有面光源
        }
    }

    RGB OptimizedPathTracerRenderer::OptTrace(const Ray& r, int currDepth){
        if (currDepth == depth) return scene.ambient.constant; //递归数目达到depth
        Vec3 origin = r.origin;
        Vec3 L_dir = Vec3{0};
        Vec3 lightPos = scene.areaLightBuffer[0].position;
        if(currDepth!=0){
            Ray shadowRay{origin, glm::normalize(lightPos - origin)};
            auto dis = glm::distance(lightPos, origin);
            auto hitRecord = closestHitObject(shadowRay);
            if(hitRecord && hitRecord->t < dis){
                L_dir = scene.areaLightBuffer[0].radiance;
            }
        }

        auto hitObject = closestHitObject(r);   //光线最近的射中物体
        auto [ t, emitted ] = closestHitLight(r);   
        // hit object
        if (hitObject && hitObject->t < t) { //hitObject在相机和面光源之间
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            auto scatteredRay = scattered.ray;
            auto attenuation = scattered.attenuation;
            auto emitted = scattered.emitted;
            auto next = trace(scatteredRay, currDepth+1);
            float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
            float pdf = scattered.pdf;
            /**
             * emitted      - Le(p, w_0)
             * next         - Li(p, w_i)
             * n_dot_in     - cos<n, w_i>
             * atteunation  - BRDF
             * pdf          - p(w)
             **/
            return emitted + attenuation * next * n_dot_in / pdf; //自发光emitted, 加上来自外部的光线的亮度
        }
        // 
        else if (t != FLOAT_INF) {  //hitObject在面光源,直接返回面光源的激发亮度
            return emitted;
        }
        else {
            return Vec3{0}; //没有hitObject,也没有面光源
        }
    }
}