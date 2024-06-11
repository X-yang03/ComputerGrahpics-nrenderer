#include "server/Server.hpp"
#include "scene/Scene.hpp"
#include "component/RenderComponent.hpp"
#include "Camera.hpp"

#include "PhotonMapper.hpp"

using namespace std;
using namespace NRenderer;

namespace PhotonMapper
{
    class Adapter : public RenderComponent
    {
        void render(SharedScene spScene) {
            PhotonMapperRenderer renderer{spScene};
            auto renderResult = renderer.render();
            auto [ pixels, width, height ]  = renderResult;
            getServer().screen.set(pixels, width, height);
            renderer.release(renderResult);
        }
    };
}

const static string description = 
    "A Simple Photon mapper. ";

REGISTER_RENDERER(PhotonMapper, description, PhotonMapper::Adapter);