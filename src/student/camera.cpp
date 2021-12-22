
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    Ray r1;
    Samplers::Rect random_rect(Vec2(aperture, aperture));
    
    r1.point.x = position.x + random_rect.sample().x;
    r1.point.y = position.y + random_rect.sample().y;
    r1.point.z = position.z;

    
    // get screen width
    float vfov = Radians(get_fov());
    float half_sh = 1.0f * tan(vfov / 2.0f);
    float sw = ( get_ar()) * 2.0f * half_sh;

    screen_coord.x = screen_coord.x * ( sw) ;
    screen_coord.y = screen_coord.y * (2.0f * half_sh);


    
    // convert ray_dir with transformation matrix iview but only the rotation and scaling not the translation 
    Vec4 ray_dir = Vec4(screen_coord.x, screen_coord.y, -1.0f * focal_dist, 0.0f);
    ray_dir = iview * ray_dir ;

    r1.dir = Vec3(ray_dir.x, ray_dir.y, ray_dir.z);

    return r1;
    //return Ray();
}
