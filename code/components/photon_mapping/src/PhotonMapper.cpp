#include "server/Server.hpp"

#include "PhotonMapper.hpp"

#include "VertexTransformer.hpp"
#include "intersections/intersections.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include "Onb.hpp"

//#include "omp.h"

#include <random>
#include <queue>

namespace PhotonMapper
{
    RGB PhotonMapperRenderer::gamma(const RGB &rgb) {
        return glm::sqrt(rgb);
    }

    void PhotonMapperRenderer::renderTask(RGBA *pixels, int width, int height, int off = 0, int step = 1) {
        
        //#pragma omp parallel for
        for (int i = off; i < height; i += step) {
            for (int j = 0; j < width; j++) {
                Vec3 color{ 0, 0, 0 };
                for (int k = 0; k < samples; k++) {
                    auto r = defaultSamplerInstance<UniformInSquare>().sample2d();
                    float rx = r.x;
                    float ry = r.y;
                    float x = (float(j) + rx) / float(width);
                    float y = (float(i) + ry) / float(height); //随机采样的光线方向
                    auto ray = camera.shoot(x, y); //打出光线
                    color += OptTrace(ray, 0); //路径追踪渲染
                    //color += trace(ray, 0); //路径追踪渲染
                }
                color /= samples; //平均
                color = gamma(color);
                pixels[(height - i - 1) * width + j] = { color, 1 };
                //fprintf(stderr, "height: %d(%d), width: %d(%d)\n", i, height, j, width);
            }
            fprintf(stderr,"height: %d(%d)\n", i, height);
        }
    }

    auto PhotonMapperRenderer::render() -> RenderResult {
        // shaders
        shaderPrograms.clear();
        ShaderCreator shaderCreator{};
        for (auto &m : scene.materials) {
            shaderPrograms.push_back(shaderCreator.create(m, scene.textures));
        }

        RGBA *pixels = new RGBA[width * height]{};

        // 局部坐标转换成世界坐标
        VertexTransformer vertexTransformer{};
        vertexTransformer.exec(spScene);
        this->bvhTree = make_shared<BVHTree>(spScene);

        for(auto &areaLight : scene.areaLightBuffer) {
            areaLight.area = glm::length(glm::cross(areaLight.u, areaLight.v));
            areaLight.normal = glm::normalize(glm::cross(areaLight.u, areaLight.v));
		}
        generatePhotonMap();

        const auto taskNums = 16;
        thread t[taskNums];
        for (int i = 0; i < taskNums; i++) {
            t[i] = thread(&PhotonMapperRenderer::renderTask,
                this, pixels, width, height, i, taskNums); //多线程渲染
        }
        for (int i = 0; i < taskNums; i++) {
            t[i].join();
        }
        //renderTask(pixels, width, height);
        getServer().logger.log("Done...");

        return { pixels, width, height };
    }

    class RandomGenerator {
    public:
        RandomGenerator()
            : generator(std::random_device{}()), distribution(0.0f, 1.0f) {}

        float generate() {
            return distribution(generator);
        }

    private:
        std::mt19937 generator;
        std::uniform_real_distribution<float> distribution;
    };


    static float random_double() {
		RandomGenerator generator;
		return generator.generate();
	}

    static bool russian_roulette(float p) {
        return random_double() < p;
    }

    void PhotonMapperRenderer::generatePhotonMap()
    {
        getServer().logger.log("Photon trace generated...");
        for (int i = 0; i < photonNum; i++)
        {
            for (auto &areaLight : scene.areaLightBuffer)
            {
                auto r1 = random_double();
                auto r2 = random_double();
                Vec3 position = areaLight.position + r1 * areaLight.u + r2 * areaLight.v;
                Vec3 randomDir_local = defaultSamplerInstance<HemiSphere>().sample3d();
                Vec3 ramdomDir_world = glm::normalize(Onb(areaLight.normal).local(randomDir_local));
                Ray Ray(position, ramdomDir_world);

                Vec3 r = (areaLight.radiance * areaLight.area) / (1.0f * photonNum * PI);
                tracePhoton(Ray, r, 0);
            }
        }
        getServer().logger.log("Photon map generated...");
        getServer().logger.log("Photon num : "+to_string(photons.size()));
        photonMap.build(photons);
        getServer().logger.log("Photon map built...");
    }

    void PhotonMapperRenderer::tracePhoton(const Ray &ray, const RGB &power, int depth)
	{
		if (depth > this->depth)
			return;
		auto hitObject = closestHitObject(ray);
		if (!hitObject)
			return;
		auto mtlHandle = hitObject->material;
		auto scattered = shaderPrograms[mtlHandle.index()]->shade(ray, hitObject->hitPoint, hitObject->normal);
		auto scatteredRay = scattered.ray;
		auto attenuation = scattered.attenuation;
		auto pdf = scattered.pdf;
		auto nextRay = scatteredRay;
        auto P_RR = 0.8f;
        pdf *= P_RR;
        if (spScene->materials[mtlHandle.index()].type == Material::LAMBERTIAN)
        {
            Photon photon{ hitObject->hitPoint, power, ray, scatteredRay, hitObject->normal };
            photons.push_back(photon);
            if (russian_roulette(P_RR))
            {
                auto cos_theta = abs(glm::dot(hitObject->normal, -ray.direction));
                auto nextPower = power * attenuation * cos_theta / pdf;
                tracePhoton(nextRay, nextPower, depth + 1);
            }
        }
        else if (spScene->materials[mtlHandle.index()].type == Material::CONDUCTOR || spScene->materials[mtlHandle.index()].type == Material::GLOSSY)
        {
            if (russian_roulette(P_RR))
            {
                auto nextPower = power * attenuation / pdf;

                Vec3 reflectedDir = glm::reflect(ray.direction, hitObject->normal);
                Ray reflectedRay(hitObject->hitPoint, reflectedDir);
                tracePhoton(reflectedRay, nextPower, depth + 1);
            }
        }
        else if (spScene->materials[mtlHandle.index()].type == Material::DIELECTRIC || spScene->materials[mtlHandle.index()].type == Material::PLASTIC)
        {
            if (russian_roulette(P_RR))
            {
                auto nextPower = power * attenuation / pdf;
                tracePhoton(nextRay, nextPower, depth + 1);
                nextPower = power * scattered.refractRatio / pdf;
                nextRay = scattered.refractionDir;
                tracePhoton(nextRay, nextPower, depth + 1);
            }
        }
	}

    void PhotonMapperRenderer::release(const RenderResult& r) {
        auto [p, w, h] = r;
        delete[] p;
    }

    HitRecord PhotonMapperRenderer::closestHitObject(const Ray& r) {
        HitRecord closestHit = nullopt;
        float closest = FLOAT_INF;
        auto hitRecord = Intersection::xBVH(r, bvhTree->root, 0.000001, closest); //BVH加速
        if (hitRecord && hitRecord->t < closest) {
            closest = hitRecord->t;
            return hitRecord;
        }
        /*for (auto& s : scene.sphereBuffer) {
            auto hitRecord = Intersection::xSphere(r, s, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& t : scene.triangleBuffer) {
            auto hitRecord = Intersection::xTriangle(r, t, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }
        for (auto& p : scene.planeBuffer) {
            auto hitRecord = Intersection::xPlane(r, p, 0.000001, closest);
            if (hitRecord && hitRecord->t < closest) {
                closest = hitRecord->t;
                closestHit = hitRecord;
            }
        }*/
        return closestHit; 
    }
    
    tuple<float, Vec3> PhotonMapperRenderer::closestHitLight(const Ray& r) {
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

    class ComPhoton {
    private:
        Vec3 position;
    public:
        ComPhoton(const Vec3 &position) : position(position) {}
        bool operator()(const Photon &p1, const Photon &p2) {
            return glm::distance(p1.position, position) < glm::distance(p2.position, position);
        }
    };


    RGB PhotonMapperRenderer::trace(const Ray& r, int currDepth) {
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
            auto pdf = scattered.pdf;
            auto P_RR = 0.8f;
            pdf *= P_RR;

            if (scene.materials[mtlHandle.index()].hasProperty("reflect"))
            {
                RGB nextReflect;
                if (russian_roulette(P_RR))
                {
                    auto reflectedDir = glm::reflect(r.direction, hitObject->normal);
                    Ray reflectedRay(hitObject->hitPoint, reflectedDir);
                    nextReflect = trace(reflectedRay, currDepth + 1);
                }
                else
                {
                    nextReflect= scene.ambient.constant;
                }
                return emitted + attenuation * nextReflect / pdf;
            }
            if(scene.materials[mtlHandle.index()].hasProperty("ior"))
			{
                RGB nextReflect, nextRefract;
                if (russian_roulette(P_RR))
                {
                    nextReflect = trace(scatteredRay, currDepth + 1);
                    nextRefract = trace(scattered.refractionDir, currDepth + 1);
                }
                else
                {
                    nextReflect = nextRefract = scene.ambient.constant;
                }
                return emitted + (attenuation * nextReflect + scattered.refractRatio * nextRefract) / pdf;
			}

            auto nearPhotons = photonMap.nearestPhotons(hitObject->hitPoint, samplePhotonNum);
            auto maxDistanceElement = 
            max_element(nearPhotons.begin(),nearPhotons.end(),
                [&hitObject](const Photon &p1, const Photon &p2) {
				    return glm::distance(p1.position, hitObject->hitPoint) < glm::distance(p2.position, hitObject->hitPoint);
			    });
			auto maxDistance = glm::distance(maxDistanceElement->position, hitObject->hitPoint);
            /*priority_queue<Photon, vector<Photon>, ComPhoton> pq(ComPhoton(hitObject->hitPoint));
            for (auto &photon : photons)
            {
                if(pq.size()<samplePhotonNum )
				{
					pq.push(photon);
				}
				else
				{
					if(glm::distance(photon.position, hitObject->hitPoint) < glm::distance(pq.top().position, hitObject->hitPoint))
					{
						pq.pop();
						pq.push(photon);
					}
				}
            }
            auto maxDistance = glm::distance(pq.top().position, hitObject->hitPoint);

            vector<Photon> nearPhotons;
            while (!pq.empty())
			{
				nearPhotons.push_back(pq.top());
				pq.pop();
			}*/

            Vec3 averageDirect{ 0,0,0 };
            RGB averageLight{ 0,0,0 };
            for (auto &photon : nearPhotons)
            {
                auto cos_theta = glm::dot(hitObject->normal, -photon.in_ray.direction);
                if (cos_theta > 0)
                {
                    averageDirect+=-photon.in_ray.direction;
                    averageLight += photon.power / (PI * maxDistance * maxDistance);
                }
            }
            auto n_dot_in = glm::dot(hitObject->normal, glm::normalize(averageDirect));
            pdf /= P_RR;
            return emitted + attenuation * averageLight * n_dot_in / pdf;
            /**
             * emitted      - Le(p, w_0)
             * next         - Li(p, w_i)
             * n_dot_in     - cos<n, w_i>
             * atteunation  - BRDF
             * pdf          - p(w)
             **/
            //return emitted + attenuation * next * n_dot_in / pdf; //自发光emitted, 加上来自外部的光线的亮度
        }
        else if (t != FLOAT_INF) {  //hitObject在面光源之后,直接返回面光源的激发亮度
            return emitted;
        }
        else {
            return Vec3{0}; //没有hitObject,也没有面光源
        }
    }

    tuple<Vec3, Vec3> PhotonMapperRenderer::sampleOnlight(const AreaLight &light) {
        Vec3 p = light.position;
        Vec3 u = light.u;
        Vec3 v = light.v;
        Vec3 normal = glm::normalize(glm::cross(u, v));
        Vec3 samplePoint = p + u * defaultSamplerInstance<UniformSampler>().sample1d() + v * defaultSamplerInstance<UniformSampler>().sample1d();
        return { samplePoint, normal };
    }

    RGB PhotonMapperRenderer::OptTrace(const Ray &r, int currDepth) {
        if (currDepth == depth) return scene.ambient.constant; //递归数目达到depth
        auto hitObject = closestHitObject(r);   //光线最近的射中物体
        auto [t, emitted] = closestHitLight(r);
        // hit object
        if (hitObject && hitObject->t < t) { //hitObject在相机和面光源之间
            auto mtlHandle = hitObject->material;
            auto scattered = shaderPrograms[mtlHandle.index()]->shade(r, hitObject->hitPoint, hitObject->normal);
            if (spScene->materials[mtlHandle.index()].type == Material::LAMBERTIAN) {
                auto scatteredRay = scattered.ray;
                auto attenuation = scattered.attenuation;
                auto emitted = scattered.emitted;

                auto [samplePoint, normal] = sampleOnlight(scene.areaLightBuffer[0]);
                Vec3 shadowRayDir = glm::normalize(samplePoint - hitObject->hitPoint);
                Ray shadowRay{ hitObject->hitPoint, shadowRayDir };
                auto shadowHit = closestHitObject(shadowRay);
                float distance2Light = glm::length(samplePoint - hitObject->hitPoint);
                float cosTheta = glm::dot(-shadowRayDir, normal);
                Vec3 L_dir;
                auto radiance = scene.areaLightBuffer[0].radiance;  //直接光照
                if (shadowHit && shadowHit->t < distance2Light || cosTheta < 0.0001) {  //如果被遮挡， 或与光源法向量夹角过小
                    L_dir = Vec3(0.f);
                }
                else {
                    float pdf_light = 1.0f / (glm::length(scene.areaLightBuffer[0].u) * glm::length(scene.areaLightBuffer[0].v)); // 光源pdf, 1/A
                    float n_dot_in_light = glm::dot(hitObject->normal, shadowRayDir);
                    Vec3 directLighting = radiance * n_dot_in_light * cosTheta / (distance2Light * distance2Light * pdf_light);
                    L_dir = attenuation * directLighting;
                }

                //auto next = OptTrace(scatteredRay, currDepth + 1);
                //if (next == radiance) next = Vec3(0.f);  //如果随机采样追踪的光线直接射到光源上, 避免二次叠加
                //float n_dot_in = glm::dot(hitObject->normal, scatteredRay.direction);
                float pdf = scattered.pdf;
                //Vec3 L_indir = attenuation * next * n_dot_in / pdf;

                auto nearPhotons = photonMap.nearestPhotons(hitObject->hitPoint, samplePhotonNum);
                auto maxDistanceElement =
                    max_element(nearPhotons.begin(), nearPhotons.end(),
                        [&hitObject](const Photon &p1, const Photon &p2) {
                            return glm::distance(p1.position, hitObject->hitPoint) < glm::distance(p2.position, hitObject->hitPoint);
                        });
                auto maxDistance = glm::distance(maxDistanceElement->position, hitObject->hitPoint);
                Vec3 averageDirect{ 0,0,0 };
                RGB averageLight{ 0,0,0 };
                for (auto &photon : nearPhotons)
                {
                    auto cos_theta = glm::dot(hitObject->normal, -photon.in_ray.direction);
                    if (cos_theta > 0)
                    {
                        averageDirect += -photon.in_ray.direction;
                        averageLight += photon.power / (PI * maxDistance * maxDistance);
                    }
                }
                auto n_dot_in = glm::dot(hitObject->normal, glm::normalize(averageDirect));
                Vec3 L_indir = attenuation * averageLight * n_dot_in / pdf;

                return emitted + L_dir + L_indir;
            }
            else if (spScene->materials[mtlHandle.index()].type == Material::DIELECTRIC) {
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if (spScene->materials[mtlHandle.index()].type == Material::CONDUCTOR) {
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(reflect, currDepth + 1);
                return reflectRGB * reflectRatio;
            }
            else if (spScene->materials[mtlHandle.index()].type == Material::PLASTIC) {
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto refract = scattered.refractionDir;
                auto refractRatio = scattered.refractRatio;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(reflect, currDepth + 1); //如果是全透射,则没有反射光线
                auto refractRGB = refractRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(refract, currDepth + 1);  //如果是全反射,则没有折射光线
                return reflectRGB * reflectRatio + refractRGB * refractRatio;
            }
            else if (spScene->materials[mtlHandle.index()].type == Material::GLOSSY) {
                auto reflect = scattered.ray;
                auto reflectRatio = scattered.attenuation;
                auto reflectRGB = reflectRatio == Vec3(0.f) ? RGB(0.f) : OptTrace(reflect, currDepth + 1);
                return reflectRGB * reflectRatio;
            }
            else {
                return Vec3(0.f);
            }
        }
        // 
        else if (t != FLOAT_INF) {  //hitObject在面光源,直接返回面光源的激发亮度
            return emitted;
        }
        else {
            return Vec3{ 0 }; //没有hitObject,也没有面光源
        }
    }
}