#include <SDL.h>
#include "core.h"
#include "display.h"

#define DEG(a) ((a) * M_PI / 180)

using namespace rr;
/*
    ds^2 = -(1 - r_g / r + r_q^2 / r^2)dt^2 + dr^2 / (1 - r_g / r + r_q^2 / r^2) + r^2 (d\theta^2 + \sin^2\theta d\phi^2)
*/

static vec3 sphericalToCartisian(const vec3 &p){
    return vec3(p.e1 * sin(p.e2) * cos(p.e3), p.e1 * sin(p.e2) * sin(p.e3), p.e1 * cos(p.e2));
}
static vec3 cartisianToSpherical(const vec3 &p){
    rrfloat r = sqrt(p.euclidLen2());
    rrfloat r2 = sqrt(p.e1*p.e1 + p.e2*p.e2);
    return vec3(r, atan2(r2, p.e3), atan2(p.e2, p.e1) + M_PI);
}

struct VelRay: public ray {
    vec3 v;
    rrfloat C;
};

class ReissnerEngine: public Engine {
    rrfloat rg, rq2, dlambda, omega;
    VelRay v1, v2;
    public:
    ReissnerEngine(rrfloat rg, rrfloat rq, rrfloat dlambda, rrfloat omega): rg(rg), rq2(rq * rq), dlambda(dlambda), omega(omega) {}
    void allocRay(unsigned int x, unsigned int y, unsigned int index, ray **r1, ray **r2){
        *r1 = &v1;
        *r2 = &v2;
    }
    int fireRay(const vec3 &pos, const vec3 &dir, ray *out) const {
        VelRay *ra = static_cast<VelRay *>(out);
        rrfloat r = pos.e1, ct = cos(pos.e2), st = sin(pos.e2);
        rrfloat cp = cos(pos.e3), sp = sin(pos.e3);
        rrfloat f = sqrt(1 - rg / r + rq2 / (r*r));

        ra->pos = sphericalToCartisian(pos);
        ra->v = vec3(
            -dir.e1 * sp - (dir.e3 * ct + dir.e2 * f * st) * cp,
            dir.e1 * cp - (dir.e3 * ct + dir.e2 * f * st) * sp,
            dir.e3 * st - dir.e2 * f * ct
        ) * omega;
        ra->C = ra->pos.euclidCross(ra->v).euclidLen2();
        return 0;
    }
    int iterateRay(unsigned int times, const ray *in1, ray *out1) const {
        const VelRay *in = static_cast<const VelRay *>(in1);
        VelRay *out = static_cast<VelRay *>(out1);

        rrfloat r = sqrt(in->pos.euclidLen2());
        rrfloat ddr = in->C / (r*r*r*r) * (- 3*rg / 2 + 2*rq2 / r);
        vec3 dir = in->pos / r;

        out->C = in->C;
        out->v = in->v + dir * ddr * dlambda;
        out->pos = in->pos + in->v * dlambda;

        return 0;
    }
};

class TexturedSphere: public Object {
    SDL_Surface *image;
    rrfloat r;
    vec3 centre;
    public:
    TexturedSphere(const char *fname, rrfloat r, const vec3 &centre): r(r), centre(centre){
        image = SDL_LoadBMP(fname);
    }
    ~TexturedSphere(){
        SDL_FreeSurface(image);
    }
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        vec3 pos1 = start->pos - centre, pos2 = end->pos - centre;
        rrfloat r1 = sqrt(pos1.euclidLen2()), r2 = sqrt(pos2.euclidLen2());
        if ((r1 < r) ^ (r2 < r)){
            rrfloat r12 = pos1.euclidLen2(), r22 = pos2.euclidLen2(), dot = pos1.euclidDot(pos2);
            rrfloat a = r12 + r22 - 2*dot, b = 2*(dot - r12), c = r12 - r*r;
            rrfloat l = (-b - sqrt(b*b - 4*a*c)) / (2*a);
            vec3 p = cartisianToSpherical(pos1 + (pos2 - pos1) * l);

            result->status = 1;
            result->distance = r > r1 ? r - r1 : r1 - r;
            color &cl = result->c;
            unsigned int x = static_cast<unsigned int>(p.e3 * image->w / (2*M_PI));
            unsigned int y = static_cast<unsigned int>((1 - cos(p.e2)) / 2 * image->h);

            Uint32 *pixel = reinterpret_cast<Uint32 *>( reinterpret_cast<char *>(image->pixels) + image->pitch * y + image->format->BytesPerPixel * x );
            SDL_GetRGB(*pixel, image->format, &cl.r, &cl.g, &cl.b);
            
            cl.a = 255;
        }
        else {
            result->status = 0;
        }
    }
};

class Sphere: public Object {
    vec3 centre;
    rrfloat r;
    color c;
    public:
    Sphere(const vec3 &centre, rrfloat r, const color &c): Object(), centre(centre), r(r), c(c){}
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        vec3 pos1 = start->pos - centre, pos2 = end->pos - centre;
        rrfloat r1 = sqrt(pos1.euclidLen2()), r2 = sqrt(pos2.euclidLen2());
        if (r1 < r && r2 > r){
            result->status = 1;
            result->distance = r - r1;
            result->c = c;
        }
        else if (r1 > r && r2 < r){
            result->status = 1;
            result->distance = r1 - r;
            result->c = c;
        }
        else {
            result->status = 0;
        }
    }
};

class StrippedSphere: public Object {
    public:
    rrfloat r;
    color c1, c2;
    rrfloat phiPatch, thetalPatch;
    vec3 centre;
    StrippedSphere(const vec3 &centre, rrfloat r, const color &c1, const color &c2, unsigned int phidiv, unsigned int thetadiv): 
        Object(), r(r), c1(c1), c2(c2), phiPatch(2*M_PI / phidiv), thetalPatch(M_PI / thetadiv), centre(centre){}
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        vec3 pos1 = start->pos - centre, pos2 = end->pos - centre;
        rrfloat r1 = sqrt(pos1.euclidLen2()), r2 = sqrt(pos2.euclidLen2());
        if ((r1 < r) ^ (r2 < r)){
            rrfloat r12 = pos1.euclidLen2(), r22 = pos2.euclidLen2(), dot = pos1.euclidDot(pos2);
            rrfloat a = r12 + r22 - 2*dot, b = 2*(dot - r12), c = r12 - r*r;
            rrfloat l = (-b - sqrt(b*b - 4*a*c)) / (2*a);
            vec3 p = cartisianToSpherical(pos1 + (pos2 - pos1) * l);
            unsigned int i = static_cast<unsigned int>(p.e2 / thetalPatch) % 2;
            unsigned int j = static_cast<unsigned int>(p.e3 / phiPatch) % 2;

            result->status = 1;
            result->c = (i ^ j) ? c1 : c2;
            result->distance = r > r1 ? r - r1 : r1 - r;
        }
        else {
            result->status = 0;
        }
    }
};

class Disc: public Object {
    public:
    rrfloat r, R;
    color c1, c2;
    rrfloat dphi;
    Disc(rrfloat r, rrfloat R, const color &c1, const color &c2, unsigned int div): r(r), R(R), c1(c1), c2(c2), dphi(2*M_PI / div){}
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        const vec3 &p1 = start->pos, &p2 = end->pos;
        if ((p1.e3 > 0) ^ (p2.e3 > 0)){
            rrfloat l = p1.e3 / (p1.e3 - p2.e3);
            vec3 p = p1 + (p2 - p1) * l;
            rrfloat r0 = sqrt(p.e1*p.e1 + p.e2*p.e2);
            if (r0 > r && r0 < R){
                rrfloat phi = atan2(p.e2, p.e1) + M_PI;
                unsigned int i = static_cast<unsigned int>(phi / dphi) % 2;
                result->status = 1;
                result->c = i ? c1 : c2;
                result->distance = sqrt((p1 - p).euclidLen2());
            }
            else {
                result->status = 0;
            }
        }
        else {
            result->status = 0;
        }
    }
};

static void animation1(unsigned int h, unsigned int w, unsigned int count, unsigned int start, rrfloat thetaStart, rrfloat thetaEnd){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0.5, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(7, M_PI / 2, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    rrfloat R = 1.27;
    StrippedSphere star(vec3(0, -1.27, 0), 0.7, color(0, 255, 0), color(0, 0, 0), 10, 5);
    renderer.renderer.addObject(&sky);
    renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 0;

    unsigned int i = start;
    c.pos.e3 = thetaStart + rrfloat(i) / count * (thetaEnd - thetaStart);

    renderer.startRender(c, [&renderer, &i, count, R, &c, &thetaEnd, &thetaStart]() -> int {
        char fname[50];
        snprintf(fname, 50, "animation1-1/t%u.bmp", i);
        renderer.saveBMP(fname);
        printf("Image animation1/t%u.bmp saved.\n", i);
        i++;
        if (i >= count){
            printf("done.\n");
            return 0;
        }
        else {
            c.pos.e3 = thetaStart + rrfloat(i) / count * (thetaEnd - thetaStart);
            renderer.clear();
            return 1;
        }
    });
}

static void test(unsigned int h, unsigned int w){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0, 0.005, 1);
    Camera c(w / rrfloat(h), 90, vec3(0.75, M_PI / 2, 0), vec3(1, 0, 0), vec3(0, -1, 0));
    WindowedRenderer renderer("hkm", &screen, &engine);
    renderer.renderer.maxSteps = 20000;

    StrippedSphere hole(vec3(0, 0, 0), 0.5, color(0, 0, 128), color(0, 0, 0), 10, 5); 
    // StrippedSphere sky(vec3(0, 0, 0), 10, color(50, 50, 50), color(50, 50, 50), 40, 20);
    TexturedSphere sky("../assets/skymap.bmp", 10, vec3(0, 0, 0));
    StrippedSphere star(vec3(-1.5, 0, 0), 0.5, color(0, 255, 0), color(0, 0, 0), 10, 5);
    renderer.renderer.addObject(&hole);
    renderer.renderer.addObject(&sky);
    // renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 1;

    unsigned int i = 0;
    renderer.startRender(c, [&renderer]() -> int {
        renderer.saveBMP("test.bmp");
        printf("Image saved\n");
        return 0;
    });
}

static int animation2(unsigned int h, unsigned int w, rrfloat startX, rrfloat endX, unsigned int count, unsigned int start){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(7, M_PI / 2, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    StrippedSphere hole(vec3(0, 0, 0), 0.5, color(0, 0, 128), color(0, 0, 0), 10, 5); 
    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    StrippedSphere star(vec3(-1.5, 0, 0), 0.7, color(0, 255, 0), color(0, 0, 0), 10, 5);
    renderer.renderer.addObject(&hole);
    renderer.renderer.addObject(&sky);
    renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 0;

    unsigned int i = start;
    star.centre.e2 = startX + rrfloat(i) / count * (endX - startX);
    renderer.startRender(c, [&renderer, &i, count, &star, &startX, &endX]() -> int {
        char fname[50];
        snprintf(fname, 50, "animation2/t%u.bmp", i);
        renderer.saveBMP(fname);
        printf("Image %s saved.\n", fname);
        i++;
        if (i >= count){
            printf("[@] done.\n");
            return 0;
        }
        else {
            star.centre.e2 = startX + rrfloat(i) / count * (endX - startX);
            renderer.clear();
            return 1;
        }
    });
}

static int animation3(unsigned int h, unsigned int w, rrfloat startY, rrfloat endY, unsigned int count, unsigned int start){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0.3, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(5, M_PI / 2, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);
    renderer.renderer.maxSteps = 20000;

    StrippedSphere sky(vec3(0, 0, 0), 10, color(50, 50, 50), color(50, 50, 50), 40, 20);
    StrippedSphere star(vec3(-1, 1, 0), 0.5, color(0, 255, 0), color(0, 0, 0), 10, 5);
    renderer.renderer.addObject(&sky);
    renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 0;

    unsigned int i = start;
    star.centre.e2 = startY + (endY - startY) * rrfloat(i) / count;
    renderer.startRender(c, [&renderer, &i, &star, count, startY, endY]() -> int {
        char fname[50];
        snprintf(fname, 50, "animation3/t%u.bmp", i++);
        renderer.saveBMP(fname);
        printf("Image %s saved.\n", fname);
        if (i >= count){
            printf("done.\n");
            return 0;
        }
        else {
            star.centre.e2 = startY + (endY - startY) * rrfloat(i) / count;
            renderer.clear();
            return 1;
        }
    });
}

static void animation4(unsigned int h, unsigned int w, unsigned int count, unsigned int start, rrfloat startY, rrfloat endY){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0.3, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(5, M_PI / 2, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    StrippedSphere star(vec3(0, 0, 0), 0.5, color(0, 255, 0), color(0, 0, 0), 10, 5);
    renderer.renderer.addObject(&sky);
    renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 0;

    unsigned int i = start;
    star.centre.e2 = startY + (endY - startY) * rrfloat(i) / count;

    renderer.startRender(c, [&renderer, &i, count, &star, startY, endY]() -> int {
        char fname[50];
        snprintf(fname, 50, "animation4/t%u.bmp", i);
        renderer.saveBMP(fname);
        printf("Image animation1/t%u.bmp saved.\n", i);
        i++;
        if (i >= count){
            printf("done.\n");
            return 0;
        }
        else {
            star.centre.e2 = startY + (endY - startY) * rrfloat(i) / count;
            renderer.clear();
            return 1;
        }
    });
}

static int test2(unsigned int h, unsigned int w){
    Screen screen(h, w);
    ReissnerEngine engine(0, 0, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(5, DEG(85), 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    StrippedSphere hole(vec3(0, 0, 0), 0.5, color(0, 0, 255), color(0, 0, 0), 10, 5);
    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    Disc d(1, 2, color(255, 255, 255), color(0, 255, 0), 20);
    renderer.renderer.addObject(&hole);
    renderer.renderer.addObject(&d);
    renderer.renderer.addObject(&sky);
    renderer.renderer.antiAlias = 1;
    renderer.startRender(c, [&renderer]() -> int {
        renderer.saveBMP("test.bmp");
        printf("Image saved.\n");
        return 0;
    });
}

static int test3(unsigned int h, unsigned int w){
    Screen screen(h, w);
    ReissnerEngine engine(10, 4, 0.01, 1);
    Camera c(w / rrfloat(h), 90, vec3(5, DEG(85), 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    StrippedSphere hole(vec3(0, 0, 0), 0.5, color(0, 0, 255), color(0, 0, 0), 10, 5);
    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    Disc d(1, 2, color(255, 255, 255), color(0, 255, 0), 20);
    renderer.renderer.addObject(&hole);
    renderer.renderer.addObject(&d);
    renderer.renderer.addObject(&sky);
    renderer.renderer.antiAlias = 1;
    renderer.startRender(c, [&renderer]() -> int {
        renderer.saveBMP("test.bmp");
        printf("Image saved.\n");
        return 0;
    });
}

int main(int argc, const char *args[]){
    unsigned int h = 400, w = 400;
    SDL_Init(SDL_INIT_VIDEO);
    {
        // animation4(h, w, 20, 0, 2, 0);
        // animation2(h, w, -2.5, 2.5, 20, 0);
        // animation3(h, w, -2, 2, 30, 0);
        test(h, w);
        // test2(h, w);
    }
    SDL_Quit();
}