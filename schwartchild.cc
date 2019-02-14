#include <SDL.h>
#include "core.h"
#include "display.h"

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
    color stripColor;
    rrfloat phiPatch, thetalPatch;
    vec3 centre;
    StrippedSphere(const vec3 &centre, rrfloat r, const color &c, unsigned int phidiv, unsigned int thetadiv): 
        Object(), r(r), stripColor(c), phiPatch(2*M_PI / phidiv), thetalPatch(M_PI / thetadiv), centre(centre){}
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        vec3 pos1 = start->pos - centre, pos2 = end->pos - centre;
        rrfloat r1 = sqrt(pos1.euclidLen2()), r2 = sqrt(pos2.euclidLen2());

        if (r1 < r && r2 > r){
            vec3 s = cartisianToSpherical(pos1);
            result->status = 1;
            result->distance = r - r1;
            unsigned int i = static_cast<unsigned int>(s.e2 / thetalPatch) % 2;
            unsigned int j = static_cast<unsigned int>(s.e3 / phiPatch) % 2;
            result->c = (i ^ j) ? color() : stripColor;
        }
        else if (r1 > r && r2 < r){
            vec3 s = cartisianToSpherical(pos1);
            result->status = 1;
            result->distance = r1 - r;
            unsigned int i = static_cast<unsigned int>(s.e2 / thetalPatch) % 2;
            unsigned int j = static_cast<unsigned int>(s.e3 / phiPatch) % 2;
            if (i ^ j){
                result->c = stripColor;
            }
            else {
                result->c = color();
            }
        }
        else {
            result->status = 0;
        }
    }
};

class Disc: public Object {

};

class PathPrinter: public Object {
    unsigned int x, y;
    public:
    PathPrinter(unsigned int x, unsigned int y): x(x), y(y) {}
    void hitTest(const ray *start, const ray *end, HitTestResult *result) const {
        result->c = 0;
        if (start->info.x == x && start->info.y == y)
            printf("(%lf, %lf, %lf)\n", start->pos.e1, start->pos.e2, start->pos.e3);
    }
};

static void animation1(unsigned int h, unsigned int w, unsigned int count, unsigned int start, rrfloat thetaStart, rrfloat thetaEnd){
    Screen screen(h, w);
    ReissnerEngine engine(0.5, 0.5, 0.01, 1);
    Camera c(3, w / rrfloat(h), 90, vec3(7, M_PI / 2, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    WindowedRenderer renderer("hkm", &screen, &engine);

    Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
    rrfloat R = 1.27;
    StrippedSphere star(vec3(0, -1.27, 0), 0.7, color(0, 255, 0), 10, 5);
    renderer.renderer.addObject(&sky);
    renderer.renderer.addObject(&star);
    renderer.renderer.antiAlias = 0;

    unsigned int i = start;
    c.pos.e3 = thetaStart + rrfloat(i) / count * (thetaEnd - thetaStart);

    renderer.startRender(c, [&renderer, &i, count, R, &c, &thetaEnd, &thetaStart]() -> int {
        char fname[50];
        snprintf(fname, 50, "animation1/t%u.bmp", i);
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

int main(int argc, const char *args[]){
    unsigned int h = 400, w = 400;
    SDL_Init(SDL_INIT_VIDEO);
    {
        animation1(h, w, 40, 2, 0, M_PI);
        // Screen screen(h, w);
        // ReissnerEngine engine(0.5, 0.5, 0.01, 1);
        // Camera c(3, w / rrfloat(h), 90, vec3(7, M_PI / 2, M_PI / 2), vec3(0, 1, 0), vec3(0, 0, 1));
        // WindowedRenderer renderer("hkm", &screen, &engine);
        // PathPrinter pp(0, 0);

        // StrippedSphere hole(vec3(0, 0, 0), 0.51, color(0, 0, 128), 10, 5); 
        // Sphere sky(vec3(0, 0, 0), 10, color(50, 50, 50));
        // StrippedSphere star(vec3(0, -1.27, 0), 0.7, color(0, 255, 0), 10, 5);
        // // renderer.renderer.addObject(&hole);
        // renderer.renderer.addObject(&sky);
        // renderer.renderer.addObject(&star);
        // renderer.renderer.antiAlias = 0;
        // // renderer.renderer.addObject(&pp);

        // unsigned int i = 0;
        // renderer.startRender(c, [&renderer]() -> int {
        //     renderer.saveBMP("test.bmp");
        //     printf("Image saved\n");
        //     return 0;
        // });
    }
    SDL_Quit();
}