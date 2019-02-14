#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <SDL.h>
#include <functional>
#include "core.h"

namespace rr {

class WindowedRenderer {
    SDL_Window *window;
    SDL_Surface *surface;
    Screen *s;
    int shouldUpdateSurface;
    public:
    int quit;
    RayRenderer renderer;
    WindowedRenderer(const char *title, Screen *s, Engine *engine);
    ~WindowedRenderer();
    void updateSurface();
    void clear();
    void startRender(const Camera &c, const std::function<int ()> &onDone);
    void saveBMP(const char *name);
};

};

#endif