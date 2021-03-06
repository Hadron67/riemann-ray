#ifndef __RR_CORE_H__
#define __RR_CORE_H__

#include <cstdlib>
#include <cstdint>
#include <cmath>
namespace rr {

typedef double rrfloat;
class RayRenderer;

struct color {
    uint8_t r, g, b, a;
    color(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255): r(r), g(g), b(b), a(a){}
};
struct ColorMixer {
    unsigned int r, g, b, a, i;
    ColorMixer(): r(0), g(0), b(0), a(0), i(0){}
    void addColor(const color &c){ i++; r += c.r; g += c.g; b += c.b; a += c.a; }
    void done(color *c){ c->r = r / i; c->g = g / i; c->b = b / i; c->a = a / i; }
};
struct colorx {
    unsigned int r, g, b, a;
    colorx(unsigned int r = 0, unsigned int g = 0, unsigned int b = 0, unsigned int a = 255): r(r), g(g), b(b), a(a){}
    colorx(const color &c): r(c.r), g(c.g), b(c.b), a(c.a){}
    colorx operator + (const colorx &cx) { return colorx(r + cx.r, g + cx.g, b + cx.b, a + cx.a); }
    colorx operator * (rrfloat f){ return colorx(r * f, g * f, b * f, a * f); }
    colorx operator / (rrfloat f){ 
        return colorx(
            static_cast<unsigned int>(r / f), 
            static_cast<unsigned int>(g / f), 
            static_cast<unsigned int>(b / f), 
            static_cast<unsigned int>(a / f)
        ); 
    }
    color toColor(){ return color(uint8_t(r), uint8_t(g), uint8_t(b), uint8_t(a)); }
};
struct vec3 {
    unsigned int patchID;
    rrfloat e1, e2, e3;
    vec3(){ patchID = 0; e1 = e2 = e3 = 0; }
    vec3(rrfloat e1, rrfloat e2, rrfloat e3, unsigned int id = 0): patchID(id), e1(e1), e2(e2), e3(e3){}
    vec3 &normalize(){
        rrfloat f = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
        e1 /= f;
        e2 /= f;
        e3 /= f;
        return *this;
    }
    vec3 euclidCross(const vec3 &v) const { 
        return vec3(e2 * v.e3 - e3 * v.e2, e3 * v.e1 - e1 * v.e3, e1 * v.e2 - e2 * v.e1, patchID);
    }
    rrfloat euclidLen2() const {
        return e1*e1 + e2*e2 + e3*e3;
    }
    rrfloat euclidDot(const vec3 &v) const {
        return e1*v.e1 + e2*v.e2 + e3*v.e3;
    }
    vec3 operator + (const vec3 &v) const { return vec3(e1 + v.e1, e2 + v.e2, e3 + v.e3, patchID); };
    vec3 operator - (const vec3 &v) const { return vec3(e1 - v.e1, e2 - v.e2, e3 - v.e3, patchID); }
    vec3 operator * (rrfloat a) const { return vec3(e1 * a, e2 * a, e3 * a, patchID); }
    vec3 operator / (rrfloat a) const { return vec3(e1 / a, e2 / a, e3 / a, patchID); }
    vec3 operator - () const { return vec3(-e1, -e2, -e3, patchID); }
};
struct RayInfo {
    unsigned int x, y;
};
struct ray {
    RayInfo info;
    vec3 pos;
};
struct Screen {
    unsigned int height, width;
    color *pixels;
    color *pixelAt(unsigned int x, unsigned int y){ return &pixels[y * width + x]; }
    Screen(unsigned h, unsigned w);
    ~Screen();
    void clear(){
        for (unsigned int i = 0; i < height * width; i++){
            pixels[i] = color();
        }
    }
};
struct Camera {
    rrfloat ratio /* = w / h */;
    vec3 pos, axis, up, across;

    Camera(rrfloat ratio, rrfloat fov, const vec3 &pos, const vec3 &dir, const vec3 &up);
};

struct HitTestResult {
    int status;
    color c;
    rrfloat distance;
};

class Object {
    Object *prev, *next;
    friend class RayRenderer;
    public:
    Object();
    virtual void hitTest(const ray *start, const ray *end, HitTestResult *result) const = 0;
};
class Engine {
    public:
    virtual void allocRay(unsigned int x, unsigned int y, unsigned int index, ray **r1, ray **r2) = 0;
    virtual int fireRay(const vec3 &pos, const vec3 &dir, ray *out) const = 0;
    virtual int iterateRay(unsigned int times, const ray *input, ray *output) const = 0;
    // virtual int calculateRay(const vec3 &pos, const vec3 &dir, color *out) const = 0;
};

class RayRenderer {
    Object *objHead;
    Screen *screen;
    Engine *engine;
    unsigned int renderY;
    const Camera *c;

    public:
    unsigned int maxSteps;
    int antiAlias;
    RayRenderer(Screen *s, Engine *e);
    void addObject(Object *obj);
    int performHitTests(const vec3 &start, const vec3 &end, color *c);
    void startRender(const Camera &c);
    void resetRender();
    int stepRender(unsigned int rows);
    private:
    void calculateOnePixel(unsigned x, unsigned int y, color *out);
    void calculateOnePixelAntialias(unsigned x, unsigned int y, color *out);
    void calculatePoint(unsigned x, unsigned int y, unsigned int index, rrfloat a, rrfloat b, color *out);
};

};

#endif