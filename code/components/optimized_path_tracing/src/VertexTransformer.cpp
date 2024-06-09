#include "VertexTransformer.hpp"
#include "glm/gtc/matrix_transform.hpp"

namespace OptimizedPathTracer
{
    struct Model {
        vector<Index> nodes;   //节点索引
        Vec3 translation = { 0, 0, 0 };   //世界坐标的位置
        Vec3 scale = { 1, 1, 1 };         //缩放
    };
    void VertexTransformer::exec(SharedScene spScene) {
        auto& scene = *spScene;
        for (auto& node : scene.nodes) {
            Mat4x4 t{1};
            Mat4x4 s{1};
            auto& model = spScene->models[node.model];
            t = glm::translate(t, model.translation);

            if (node.type == Node::Type::TRIANGLE) {
                for (int i=0; i<3; i++) {
                    auto& v = scene.triangleBuffer[node.entity].v[i];
                    v = t*Vec4{v, 1};
                }
            }
            else if (node.type == Node::Type::SPHERE) {
                auto& v = scene.sphereBuffer[node.entity].position;
                v = t*Vec4{v, 1};
            }
            else if (node.type == Node::Type::PLANE) {
                auto& v = scene.planeBuffer[node.entity].position;
                v = t*Vec4{v, 1};
            }
            else if (node.type == Node::Type::MESH) {
                s = glm::scale(s, model.scale);
                t = t * s; //复合矩阵变换
                auto& mesh = scene.meshBuffer[node.entity];
                for (int i=0; i<mesh.positions.size(); i++) {
                    mesh.positions[i] = t*Vec4{mesh.positions[i], 1};
                }
            }
        }
    }
}