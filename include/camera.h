#ifndef CAMERA_H
#define CAMERA_H

#include "rtweekend.h"

class camera {
    public:
        camera(
            point3 lookfrom,
            point3 lookat,
            vec3 vup,
            double vfov,    // vertical filed-of-view in degress
            double aspect_ratio,
            double aperture,    // 光圈 这里是光圈的直径？
            double focus_dist,  // 应该就是camera到屏幕的距离
                                // 之前是camera(0,0,0) -> (0,0,-1) 现在
                                // 我们让光线出发到z= -focus_dist 平面
            double _time0 = 0,
            double _time1 = 0
        ) {
            auto theta = degrees_to_radians(vfov);
            auto h = tan(theta/2);
            auto viewport_height = 2.0 * h;
            auto viewport_width = aspect_ratio * viewport_height;
            
            w = unit_vector(lookfrom - lookat);
            u = unit_vector(cross(vup, w));
            v = cross(w, u);

            origin = lookfrom;
            horizontal = focus_dist * viewport_width * u;   // 真正的宽度(乘了focus_dist后的结果)
            vertical = focus_dist * viewport_height * v;
            lower_left_corner = origin - horizontal/2 - vertical/2 - focus_dist * w;

            lens_radius = aperture / 2;
            time0 = _time0;
            time1 = _time1;
        }

        ray get_ray(double s, double t) const {
            // return ray(origin, lower_left_corner + s*horizontal + t*vertical - origin);
            vec3 rd = lens_radius * random_in_unit_disk();
            vec3 offset = u * rd.x() + v * rd.y();  
                    // 偏置向量在x y 方向的对应坐标乘以 x y 方向上的单位向量得到 真实偏置向量offset

            return ray(
                origin + offset,    // 随机偏置后的光线出发点
                lower_left_corner + s*horizontal + t*vertical - origin - offset,
                //                          屏幕上某像素的坐标  - 出发点坐标  ==  光线方向
                random_double(time0, time1) // 随机时间发出光线
            );
        }

    private:
        point3 origin;
        point3 lower_left_corner;
        vec3 horizontal;
        vec3 vertical;
        vec3 u, v, w;
        double lens_radius;
        double time0, time1;    //shutter open/close times
};
#endif
