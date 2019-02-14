#include "display.h"
using namespace rr;

struct threadData {
    WindowedRenderer *renderer;
    const std::function<int ()> &onDone;
    threadData(WindowedRenderer *r, const std::function<int ()> &onDone): renderer(r), onDone(onDone){}
};

static int calculateThread(void *ptr){
    threadData *data = reinterpret_cast<threadData *>(ptr);
    WindowedRenderer *renderer = data->renderer;
    do {
        unsigned int i = 0;
        while (!renderer->quit && renderer->renderer.stepRender(1)){
            renderer->updateSurface();
            printf("%u\n", i++);
        }
        renderer->renderer.resetRender();
    } while(!renderer->quit && data->onDone());
    return 0;
}

WindowedRenderer::WindowedRenderer(const char *title, Screen *s, Engine *engine): s(s), shouldUpdateSurface(0), renderer(s, engine) {
    window = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, s->width, s->height, 0);
    surface = SDL_GetWindowSurface(window);
}
WindowedRenderer::~WindowedRenderer(){
    SDL_DestroyWindow(window);
}

void WindowedRenderer::updateSurface(){
    Uint32 *pixels = reinterpret_cast<Uint32 *>(surface->pixels);
    SDL_LockSurface(surface);
    for (unsigned int x = 0; x < s->width * s->height; x++){
        color *c = &s->pixels[x];
        pixels[x] = SDL_MapRGBA(surface->format, c->r, c->g, c->b, c->a);
        // pixels[x] = SDL_MapRGBA(surface->format, 255, 255, 255, c->a);
    }
    shouldUpdateSurface = 1;
    SDL_UnlockSurface(surface);
}

void WindowedRenderer::clear(){
    s->clear();
    updateSurface();
}

void WindowedRenderer::startRender(const Camera &c, const std::function<int ()> &onDone){
    renderer.startRender(c);
    quit = 0;
    threadData tdata(this, onDone);
    SDL_Thread *t = SDL_CreateThread(calculateThread, "calculate thread", reinterpret_cast<void *>(&tdata));

    SDL_Event e;
    while (!quit){
        while (SDL_PollEvent(&e)){
            switch (e.type){
                case SDL_QUIT:
                    quit = 1;
                    break;
                default:;
            }
        }
        SDL_LockSurface(surface);
        SDL_UpdateWindowSurface(window);
        SDL_UnlockSurface(surface);
        SDL_Delay(50);
    }
    int s = 0;
    SDL_WaitThread(t, &s);
}

void WindowedRenderer::saveBMP(const char *name){
    SDL_SaveBMP(surface, name);
}