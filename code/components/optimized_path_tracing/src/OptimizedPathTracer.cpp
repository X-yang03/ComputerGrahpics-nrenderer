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

        std::string vertexstr = "min: " + std::to_string(bvhTree->root->_min.x) + " " + std::to_string(bvhTree->root->_min.y) + " " + std::to_string(bvhTree->root->_min.z) + " max: " + std::to_string(bvhTree->root->_max.x) + " " + std::to_string(bvhTree->root->_max.y) + " " + std::to_string(bvhTree->root->_max.z);
        getServer().logger.log(vertexstr);
        int node_num = bvhTree->aabbs.size();
        std::string str= "BVH Tree has " + std::to_string(node_num) + " nodes.";  
        getServer().logger.log(str);

        const auto taskNums = 8;
        thread t[taskNums];
        for (int i=0; i < taskNums; i++) {
            t[i] = thread(&OptimizedPathTracerRenderer::renderTask,
                this, pixels, width, height, i, taskNums); //多线程渲染
        }
        for(int i=0; i < taskNums; i++) {
            t[i].join();
        }
        getServer().logger.log("Done...");
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
            return hitRecord1;
            closest1 = hitRecord1->t;
            std::string str = "closest1: " + std::to_string(closest1);
            //getServer().logger.log(str);
            closestHit = hitRecord1;

        }

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
        // if(closest1 == closest){
        //     std::string str = "closest1: " + std::to_string(closest1) + " closest: " + std::to_string(closest);
        //     getServer().logger.log(str);
        // }
        return closestHit; 
    }
    
    tuple<float, Vec3> OptimizedPathTracerRenderer::closestHitLight(const Ray& r) {
        Vec3 v = {};
        HitRecord closest = getHitRecord(FLOAT_INF, {}, {}, {});  //使用INF初始化,表示最近交点在无穷远处
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
        else if (t != FLOAT_INF) {  //hitObject在面光源之后,直接返回面光源的激发亮度
            return emitted;
        }
        else {
            return Vec3{0}; //没有hitObject,也没有面光源
        }
    }
}