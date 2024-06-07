#include "server/Server.hpp"
#include "scene/Scene.hpp"
#include "component/RenderComponent.hpp"
#include "Camera.hpp"

#include "OptimizedPathTracer.hpp"

using namespace std;
using namespace NRenderer;

namespace OptimizedPathTracer
{
    class Adapter : public RenderComponent
    {
        void render(SharedScene spScene) {
            OptimizedPathTracerRenderer renderer{spScene};
            auto renderResult = renderer.render();
            auto [ pixels, width, height ]  = renderResult;
            getServer().screen.set(pixels, width, height);
            renderer.release(renderResult);
        }
    };
}

const static string description = 
    "Optimized Path Tracer. "
    "Using BVH to accelerate rendering."
    "\nPlease use scene file : cornel_area_light.scn";

REGISTER_RENDERER(OptimizedPathTracer, description, OptimizedPathTracer::Adapter);