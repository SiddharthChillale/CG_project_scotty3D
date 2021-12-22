
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    auto origin = ray.point;
    auto dot_p = dot(origin, ray.dir);
    float Delta = dot_p * dot_p - dot(origin, origin) + radius*radius;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float valid_t = 0.0f;

    if(Delta >= 0.0f) {
       
        t1 = (-1.0f * dot_p) - sqrt(Delta);
        t2 = (-1.0f * dot_p) + sqrt(Delta);
        //assert(t1 >= 0.0f);
        //assert(t2 >= 0.0f);

        if(t1 <= t2) {

            if(t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y) {
                valid_t = t1;
            } else if(t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y) {
                valid_t = t2;
            }
        }
        else {
            if(t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y) {
                valid_t = t2;
            } 
            else if(t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y) {
                valid_t = t1;
            }
        }

        ray.dist_bounds.y = valid_t;
        
        Trace ret;
        ret.origin = ray.point;
        ret.hit = true;                                  // was there an intersection?
        ret.distance =  valid_t ;                         // at what distance did the intersection occur?
        ret.position = (ray.point + (valid_t * ray.dir)) ; // where was the intersection?
        
        ret.normal =  ret.position.unit(); // what was the surface normal at the intersection?
        
        return ret;

    } else {
        Trace ret;
        ret.origin = ray.point;
        ret.hit = false;                  // was there an intersection?
        ret.distance = 0.0f;              // at what distance did the intersection occur?
        ret.position = Vec3{};            // where was the intersection?
        ret.normal = Vec3{};              // what was the surface normal at the intersection?
        return ret;
    }
    /*if(valid_t != 0.0f) {

        OutputDebugStringW(L"\n not zero ");
    }*/
}

} // namespace PT
