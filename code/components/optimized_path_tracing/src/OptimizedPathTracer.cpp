#include "server/Server.hpp"

#include "OptimizedPathTracer.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"

namespace OptimizedPathTracer
{
    float gaussian(float x, float sigma) {
    return std::exp(-(x * x) / (2 * sigma * sigma));
    }

    void bilateralFilter(RGBA* pixels, int width, int height){
        float sigmaSpatial = 2.0f;
        float sigmaRange = 0.1f;
        int radius = 3;
        RGB* result = new RGB[width * height]{};
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Vec3 sum = {0, 0, 0};
                float weightSum = 0;

                Vec3 centerColor = pixels[y * width + x];

                for (int ky = -radius; ky <= radius; ++ky) {
                    for (int kx = -radius; kx <= radius; ++kx) {
                        int nx = x + kx;
                        int ny = y + ky;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            Vec3 neighborColor = pixels[ny * width + nx];

                            float spatialWeight = gaussian(std::sqrt(kx * kx + ky * ky), sigmaSpatial);
                            float rangeWeight = gaussian(std::sqrt((neighborColor - centerColor).r * (neighborColor - centerColor).r + 
                                                                (neighborColor - centerColor).g * (neighborColor - centerColor).g + 
                                                                (neighborColor - centerColor).b * (neighborColor - centerColor).b), sigmaRange);

                            float weight = spatialWeight * rangeWeight;
                            sum = sum + neighborColor * weight;
                            weightSum += weight;
                        }
                    }
                }

                result[y * width + x] = sum * (1.0f / weightSum);
            }
         }
          for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                pixels[y * width + x] = {result[y * width + x],1};
            }
          }

    }


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
                    color += OptTrace(ray, 0); //路径追踪渲染
                }
                color /= samples; //平均
                color = gamma(color);
                pixels[(height-i-1)*width+j] = {color, 1};
            }
        }
        //bilateralFilter(pixels, width, height);
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
        auto hitRecord = Intersection::xBVH(r, bvhTree->root, 0.000001, closest); //BVH加速
        if (hitRecord && hitRecord->t < closest ) {
            closest = hitRecord->t;
            return hitRecord;
        }
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

    tuple<Vec3, Vec3> OptimizedPathTracerRenderer::sampleOnlight(const AreaLight& light){
        Vec3 p = light.position;
        Vec3 u = light.u;
        Vec3 v = light.v;
        Vec3 normal = glm::normalize(glm::cross(u, v));
        Vec3 samplePoint = p + u * defaultSamplerInstance<UniformSampler>().sample1d() + v * defaultSamplerInstance<UniformSampler>().sample1d();
        return {samplePoint, normal};
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

                auto [samplePoint, normal] = sampleOnlight(scene.areaLightBuffer[0]);
                Vec3 shadowRayDir = glm::normalize(samplePoint - hitObject->hitPoint);
                Ray shadowRay{hitObject->hitPoint, shadowRayDir};
                auto shadowHit = closestHitObject(shadowRay);
                float distance2Light = glm::length(samplePoint - hitObject->hitPoint);
                float cosTheta = glm::dot(-shadowRayDir, normal);
                Vec3 L_dir;
                auto radiance = scene.areaLightBuffer[0].radiance;  //直接光照
                if(shadowHit && shadowHit->t < distance2Light || cosTheta < 0.0001){  //如果被遮挡， 或与光源法向量夹角过小
                    L_dir = Vec3(0.f);
                }
                else{
                    float pdf_light = 1.0f / (glm::length(scene.areaLightBuffer[0].u) * glm::length(scene.areaLightBuffer[0].v)); // 光源pdf, 1/A
                    float n_dot_in_light = glm::dot(hitObject->normal, shadowRayDir); 
                    Vec3 directLighting = radiance * n_dot_in_light * cosTheta / (distance2Light * distance2Light * pdf_light);
                    L_dir = attenuation * directLighting;
                }
                
                auto next = OptTrace(scatteredRay, currDepth+1);
                if(next == radiance) next = Vec3(0.f);  //如果随机采样追踪的光线直接射到光源上, 避免二次叠加
                float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
                float pdf = scattered.pdf;
                Vec3 L_indir = attenuation * next * n_dot_in / pdf;

                return emitted + L_dir + L_indir;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::DIELECTRIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::CONDUCTOR){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); 
                return reflectRGB * reflectRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::PLASTIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::GLOSSY){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); 
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
    
    RGB OptimizedPathTracerRenderer::ProbablityTrace(const Ray& r, int currDepth){
        if (currDepth == depth) return scene.ambient.constant; //递归数目达到depth
        auto hitObject = closestHitObject(r);   //光线最近的射中物体
        auto [ t, emitted ] = closestHitLight(r);   
        float stop_prob = 0.85f;
        // hit object
        if (hitObject && hitObject->t < t) { //hitObject在相机和面光源之间
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            if(spScene->materials[mtlHandle.index()].type == Material::LAMBERTIAN){
                auto scatteredRay = scattered.ray;
                auto attenuation = scattered.attenuation;
                auto emitted = scattered.emitted;

                auto [samplePoint, normal] = sampleOnlight(scene.areaLightBuffer[0]);
                Vec3 shadowRayDir = glm::normalize(samplePoint - hitObject->hitPoint);
                Ray shadowRay{hitObject->hitPoint, shadowRayDir};
                auto shadowHit = closestHitObject(shadowRay);
                float distance2Light = glm::length(samplePoint - hitObject->hitPoint);
                float cosTheta = glm::dot(-shadowRayDir, normal);
                Vec3 L_dir;
                auto radiance = scene.areaLightBuffer[0].radiance;  //直接光照
                if(shadowHit && shadowHit->t < distance2Light || cosTheta < 0.0001){  //如果被遮挡， 或与光源法向量夹角过小
                    L_dir = Vec3(0.f);
                }
                else{
                    float pdf_light = 1.0f / (glm::length(scene.areaLightBuffer[0].u) * glm::length(scene.areaLightBuffer[0].v)); // 光源pdf, 1/A
                    float n_dot_in_light = glm::dot(hitObject->normal, shadowRayDir); 
                    Vec3 directLighting = radiance * n_dot_in_light * cosTheta / (distance2Light * distance2Light * pdf_light);
                    L_dir = attenuation * directLighting;
                }
                if(defaultSamplerInstance<UniformSampler>().sample1d() < stop_prob) //随机终止
                    return emitted + L_dir;
                auto next = OptTrace(scatteredRay, currDepth+1);
                if(next == radiance) next = Vec3(0.f);  //如果随机采样追踪的光线直接射到光源上, 避免二次叠加
                float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
                float pdf = scattered.pdf;
                Vec3 L_indir = attenuation * next * n_dot_in / pdf;

                return emitted + L_dir + L_indir/(1-stop_prob);
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::DIELECTRIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::CONDUCTOR){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); 
                return reflectRGB * reflectRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::PLASTIC){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if(spScene->materials[mtlHandle.index()].type == Material::GLOSSY){
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) :OptTrace(reflect, currDepth + 1); 
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
}