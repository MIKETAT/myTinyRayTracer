#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "vec3.h"
#include "onb.h"
#include "pdf.h"

class sphere : public hittable {
    public:
        // constructor
        sphere() {}
        sphere(point3 cen, double r, shared_ptr<material> m)
         : center(cen), radius(r), mat_ptr(m) {};

        virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;
                                // override 表示当前重写了基类的虚函数, 并强制编译器检查
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

        double pdf_value(const point3& o, const vec3& v) const;
        vec3 random(const point3& o) const;

    public:
        point3 center;
        double radius;
        shared_ptr<material> mat_ptr;   //指向材质
    private:
        static void get_sphere_uv(const point3& p, double& u, double& v) {
            // p: a given point on the sphere of radius one, centered at the origin.
            // u: returned value [0, 1] of angle around the Y axis from X = -1
            // v: returned value [0, 1] of angle from Y=-1 to Y= +1
            //      <1 0 0> yields <0.50 0.50>      <-1 0 0> yields <0.00 0.50>
            //      <0 1 0> yields <0.50 1.00>      <0 -1 0> yields <0.50 0.00>
            //      <0 0 1> yields <0.25 0.50>      <0 0 -1> yields <0.75 0.50>

            auto theta = acos(-p.y());
            auto phi = atan2(-p.z(), p.x()) + pi;

            u = phi / (2*pi);
            v = theta / pi;
        }
};

bool sphere::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());   // b*(A-C) 
    auto c = oc.length_squared() - radius*radius;

    auto discriminant = half_b*half_b - a*c;
    if (discriminant < 0)       return false;
    auto sqrted = sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range 找到接受范围内最近的根
    auto root = (-half_b - sqrted) / a;
    if (root < t_min || root > t_max) {
        root = (-half_b + sqrted) / a;
        if (root < t_min || root > t_max) { //只有更近的交点（更先会被击中）才会保留下来 
            return false;                   // 后面击中的交点 时间 t > 前面交点的时间t_max 不会被保留  
        }
    }

    rec.t = root;   //第一次击中球体的时间
    rec.p = r.at(rec.t);    // 交点位置
    vec3 outward_normal = (rec.p - center) / radius;
    rec.set_face_normal(r, outward_normal);
    get_sphere_uv(outward_normal, rec.u, rec.v);
    rec.mat_ptr = mat_ptr;
    
    return true;
}

bool sphere::bounding_box(double time0, double time1, aabb& output_box) const {
    output_box = aabb(
        center - vec3(radius, radius, radius),
        center + vec3(radius, radius, radius));
        return true;
}

double sphere::pdf_value(const point3& o, const vec3& v) const {
    hit_record rec;
    if (!this->hit(ray(o, v), 0.001, infinity, rec))
        return 0;
    
    auto cos_theta_max = sqrt(1 - radius*radius/(center-o).length_squared());
    auto solid_angle = 2*pi*(1 - cos_theta_max);

    return 1 / solid_angle;
}

vec3 sphere::random(const point3& o) const {
    vec3 direction = center - o;
    auto distance_squared = direction.length_squared();
    onb uvw;
    uvw.build_from_w(direction);
    return uvw.local(random_to_sphere(radius, distance_squared));
}

#endif