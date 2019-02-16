#include <new>
#include <cstdlib>
#include <cstdio>
#include "core.h"

using namespace rr;
Screen::Screen(unsigned h, unsigned w): height(h), width(w){
    pixels = reinterpret_cast<color *>(malloc(sizeof(color) * h * w));
    for (unsigned int i = 0; i < h * w - 1; i++){
        new (&pixels[i]) color();
    }
}
Screen::~Screen(){
    free(pixels);
    // color has no destructor
}

Camera::Camera(rrfloat ratio, rrfloat fov, const vec3 &pos, const vec3 &dir, const vec3 &up): ratio(ratio), pos(pos){
    this->axis = dir;
    this->axis.normalize();
    rrfloat a = tan(fov / 2 * M_PI / 180);
    vec3 n = (up - this->axis * this->axis.euclidDot(up)).normalize();
    this->up = n * a;
    across = this->axis.euclidCross(n) * a * ratio;
}

Object::Object(): prev(nullptr), next(nullptr) {}

RayRenderer::RayRenderer(Screen *s, Engine *e): objHead(nullptr), screen(s), engine(e), maxSteps(10000), antiAlias(0){}

void RayRenderer::addObject(Object *obj){
    if (objHead != nullptr){
        obj->prev = obj;
    }
    obj->next = objHead;
    objHead = obj;
}

void RayRenderer::calculateOnePixel(unsigned x, unsigned int y, color *out){
    rrfloat a = rrfloat(x) / screen->width - 0.5, b = .5 - rrfloat(y) / screen->height;
    calculatePoint(x, y, 0, a, b, out);
}

void RayRenderer::calculateOnePixelAntialias(unsigned x, unsigned int y, color *out){
    rrfloat a = rrfloat(x) / screen->width - 0.5, b = .5 - rrfloat(y) / screen->height;
    rrfloat da = .25 / screen->width, db = .25 / screen->height;
    ColorMixer mixer;
    color c;
    calculatePoint(x, y, 0, a - da, b - db, &c);
    mixer.addColor(c);
    calculatePoint(x, y, 1, a - da, b + db, &c);
    mixer.addColor(c);
    calculatePoint(x, y, 2, a + da, b + db, &c);
    mixer.addColor(c);
    calculatePoint(x, y, 3, a + da, b - db, &c);
    mixer.addColor(c);
    mixer.done(out);
}

void RayRenderer::calculatePoint(unsigned x, unsigned int y, unsigned int index, rrfloat a, rrfloat b, color *out){
    const Camera &c = *this->c;
    
    vec3 dir = (c.axis + c.across * a + c.up * b).normalize();
    HitTestResult hresult;
    rrfloat distance = 0;

    ray *start, *end;
    engine->allocRay(x, y, index, &start, &end);
    start->info.x = end->info.x = x;
    start->info.y = end->info.y = y;
    engine->fireRay(c.pos, dir, start);
    engine->iterateRay(0, start, end);
    unsigned int times = 0;
    while (times < maxSteps){
        int found = 0;
        for (Object *obj = objHead; obj != nullptr; obj = obj->next){
            obj->hitTest(start, end, &hresult);
            if (hresult.status && (!found || hresult.distance < distance)){
                found = 1;
                *out = hresult.c;
                distance = hresult.distance;
            }
        }
        if (found)
            return;
        engine->iterateRay(times++, end, start);
        ray *r = end;
        end = start;
        start = r;
    }
    *out = hresult.c;
}

void RayRenderer::startRender(const Camera &c){
    renderY = 0;
    this->c = &c;
}
void RayRenderer::resetRender(){
    renderY = 0;
}
int RayRenderer::stepRender(unsigned int rows){
    while (renderY < screen->height && rows--){
        for (unsigned int x = 0; x < screen->width; x++){
            if (!antiAlias)
                calculateOnePixel(x, renderY, screen->pixelAt(x, renderY));
            else
                calculateOnePixelAntialias(x, renderY, screen->pixelAt(x, renderY));
        }
        renderY++;
    }
    return renderY < screen->height;
}
