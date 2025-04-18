package raytracer

import "core:fmt"
import "core:sync"
import "core:math"
import "core:slice"
import "core:testing"
import "core:math/rand"
import "core:math/linalg"
import "core:time"

dot :: linalg.dot

Plane :: struct { normal: [3]f32 }
Ellipsoid :: struct { radii: [3]f32 }
Box :: struct { extent: [3]f32 }

Geometry :: union { Plane, Ellipsoid, Box }

Object :: struct {
    geometry: Geometry,
    pos: [3]f32,
    rotation: quaternion128,
    color: [3]f32,
    emission: [3]f32,
    material_kind: enum u32 {
        Diffuse, Metallic, Dielectric,
    },
    ior: f32,
}

/*
Point_Light :: struct {
    pos: [3]f32,
    attenuation: [3]f32,
}

Directed_Light :: struct {
    direction: [3]f32,
}

Light_Source :: struct #raw_union {
    point: Point_Light,
    directed: Directed_Light,
}

Light :: struct {
    source: Light_Source,
    kind: enum u32 {
        Point,
        Directed,
    },
    intensity: [3]f32,
}
*/

Cam :: struct {
    pos: [3]f32,
    basis: matrix[3, 3]f32,
    fov_x: f32,
}

Scene :: struct {
    cam: Cam,
    // ambient: [3]f32,
    // lights: [dynamic]Light,
    objects: [dynamic]Object,
    light_surfaces: [dynamic]Object,
}

finish_scene :: proc(s: ^Scene) {
    for object, i in s.objects {
        if _, is_plane := object.geometry.(Plane); !is_plane && norm_l1(object.emission) > 1e-6 {
            append(&s.light_surfaces, object)
        }
    }
}

destory_scene :: proc(s: Scene) {
    delete(s.objects)
    delete(s.light_surfaces)
}

Ray :: struct {
    o, d: [3]f32,
}

// If t is negative, the ray does *not* intersect the geometry,
// and other fields have arbitrary values
Geometry_Hit :: struct {
    inside: bool,
    n, p: [3]f32,
    t: f32,
}

intersect_ray_plane :: proc(ray: Ray, plane: Plane) -> (hit: Geometry_Hit) {
    using hit
    n = plane.normal
    o_dot_n := dot(ray.o, n)
    t = -o_dot_n / dot(ray.d, n)
    inside = o_dot_n < 0
    p = ray.o + t * ray.d
    return
}

intersect_ray_ellipsoid :: proc(ray: Ray, e: Ellipsoid) -> (hit: Geometry_Hit) {
    using hit
    o_div_r := ray.o / e.radii
    d_div_r := ray.d / e.radii
    a := dot(d_div_r, d_div_r)
    b := dot(o_div_r, d_div_r)
    c := dot(o_div_r, o_div_r) - 1
    discriminant := b * b - a * c
    if discriminant < 0 {
        t = -1
        return
    }
    discriminant_root := math.sqrt(discriminant)
    t1 := (-b - discriminant_root) / a
    t2 := (-b + discriminant_root) / a
    fmt.assertf(!(t2 < t1), "%v %v", t1, t2)
    inside = t1 < 0
    t = t2 if inside else t1
    p = ray.o + t * ray.d
    n = linalg.normalize(p / (e.radii * e.radii))
    return
}

intersect_ray_box :: proc(ray: Ray, box: Box) -> (hit: Geometry_Hit) {
    using hit
    t1_raw := (box.extent - ray.o) / ray.d
    t2_raw := (-box.extent - ray.o) / ray.d
    t_min := linalg.min(t1_raw, t2_raw)
    t_max := linalg.max(t1_raw, t2_raw)
    t1 := max(t_min.x, t_min.y, t_min.z)
    t2 := min(t_max.x, t_max.y, t_max.z)
    if t1 > t2 {
        t = -1
        return
    }
    inside = t1 < 0
    t = t2 if inside else t1
    p = ray.o + t * ray.d
    n = linalg.step(box.extent, linalg.abs(p) + 1e-4) * linalg.sign(p)
    return
}

Hit :: struct {
    object_id: int,
    t: f32,
    p, n: [3]f32,
    inside: bool,
}

cast_ray :: proc(objects: []Object, ray: Ray, max_dist: f32) -> (hit: Hit) {
    hit.object_id = 0
    hit.t = max_dist
    RAY_EPS :: 1e-3

    // assert(abs(linalg.length2(ray.d) - 1) < 1e-4)

    for object, i in objects {
        if i == 0 do continue

        conj_rotation := conj(object.rotation)
        local_d := linalg.mul(conj_rotation, ray.d)
        local_o := linalg.mul(conj_rotation, ray.o - object.pos) + local_d * RAY_EPS
        local_ray := Ray{o = local_o, d = local_d}

        gh: Geometry_Hit = ---
        switch geometry in object.geometry {
        case Plane:
            gh = intersect_ray_plane(local_ray, geometry)
        case Ellipsoid:
            gh = intersect_ray_ellipsoid(local_ray, geometry)
        case Box:
            gh = intersect_ray_box(local_ray, geometry)
        }
        if gh.t > 0 && gh.t < hit.t {
            hit = {
                t = gh.t, n = linalg.mul(object.rotation, gh.n),
                object_id = i, inside = gh.inside,
                // p is set later
            }
        }
    }

    hit.t += RAY_EPS
    hit.p = ray.o + hit.t * ray.d

    return
}

raytrace :: proc(scene: Scene, ray: Ray, depth_left: i32) -> (exitance: [3]f32) {
    if depth_left == 0 do return 0

    hit := cast_ray(scene.objects[:], ray, math.INF_F32)

    debug_log_ray({
        ray = ray,
        t = hit.t,
        level = depth_left,
    })

    if hit.inside do hit.n = -hit.n

    object := scene.objects[hit.object_id]

    if hit.object_id == 0 {
        return object.color
    }

    switch object.material_kind {
    case .Diffuse:
        // total_intensity: [3]f32 = scene.ambient

        /*
        for light in scene.lights[1:] {
            dist: f32 = math.INF_F32
            d: [3]f32 = ---
            intensity: [3]f32 = ---

            switch light.kind {
            case .Point:
                point_light := light.source.point
                dist = linalg.distance(hit.p, point_light.pos)
                d = linalg.normalize(point_light.pos - hit.p)
                attenuation := light.source.point.attenuation
                intensity = light.intensity / (
                    attenuation.x +
                    attenuation.y * dist +
                    attenuation.z * dist * dist \
                )
            case .Directed:
                directed_light := light.source.directed
                d = directed_light.direction
                intensity = light.intensity
            }

            light_ray := Ray{o = hit.p, d = d}
            light_hit := cast_ray(scene, light_ray, dist)

            debug_log_ray({
                ray = light_ray,
                t = light_hit.t,
                level = -1,
            })

            if light_hit.object_id == 0 {
                total_intensity += intensity * max(dot(hit.n, d), 0)
            }
        }
        */

        // exitance = total_intensity * object.color

        // reflected := halfsphere_uniform(hit.n)
        // exitance = raytrace(scene, Ray{hit.p, reflected}, depth_left - 1)
        // exitance *= 2 * object.color * linalg.dot(reflected, hit.n)

        cosine_weighted_weight: f32 = len(scene.light_surfaces) > 0 ? 0.5 : 1.0
        reflected: [3]f32
        if rand.float32() <= cosine_weighted_weight {
            reflected = cosine_weighted(hit.n)
        } else {
            reflected = surface_sampling(scene.light_surfaces[:], hit.p)
        }
        pdf := len(scene.light_surfaces) > 0 ? math.lerp(
            surface_sampling_pdf(scene.light_surfaces[:], hit.p, reflected),
            cosine_weighted_pdf(hit.n, reflected),
            cosine_weighted_weight,
        ) : cosine_weighted_pdf(hit.n, reflected)
        cosine := linalg.dot(reflected, hit.n)
        if pdf > 1e-6 && cosine > 0 {
            irradiance := raytrace(scene, Ray{hit.p, reflected}, depth_left - 1)
            exitance = object.color / math.PI * irradiance * cosine / pdf
        }

    case .Metallic:
        reflected := ray.d - 2 * dot(hit.n, ray.d) * hit.n

        irradiance: [3]f32
        irradiance = raytrace(scene, Ray{o = hit.p, d = reflected}, depth_left - 1)

        exitance = object.color * irradiance

    case .Dielectric:
        rel_ior := object.ior if hit.inside else 1 / object.ior
        cos_theta1 := -dot(hit.n, ray.d)
        sin_theta2 := math.sqrt(1 - sq(cos_theta1)) * rel_ior

        if sin_theta2 < 1 {
            base_reflection := sq((rel_ior - 1) / (rel_ior + 1))
            ratio := base_reflection + (1 - base_reflection) * math.pow(1 - cos_theta1, 5)
            cos_theta2 := math.sqrt(1 - sq(sin_theta2))
            refracted := ray.d * rel_ior + hit.n * (rel_ior * cos_theta1 - cos_theta2)
            refracted = linalg.normalize(refracted)
            reflected := ray.d + 2 * cos_theta1 * hit.n

            color_coef := object.color if !hit.inside else 1

            if rand.float32() > ratio {
                exitance = color_coef * raytrace(scene, Ray{o = hit.p, d = refracted}, depth_left - 1)
            } else {
                exitance = raytrace(scene, Ray{o = hit.p, d = reflected}, depth_left - 1)
            }
        } else {
            reflected := ray.d + 2 * cos_theta1 * hit.n
            exitance = raytrace(scene, Ray{
                o = hit.p,
                d = reflected,
            }, depth_left - 1)
        }
    }

    exitance += object.emission

    return exitance
}

@(thread_local) debug_info: struct {
    rc: Rc,
    pixel: [2]u32,
}

render_scene :: proc(rc: Rc, scene: Scene, number_of_trials: int = 1) {
    dims_f32 := linalg.to_f32(rc.dims)
    aspect_ratio := dims_f32.x / dims_f32.y
    tan_fov_x := math.tan(scene.cam.fov_x / 2)
    tan_fov_y := tan_fov_x / aspect_ratio

    pixel_to_ray_dir :=
        linalg.matrix4_from_matrix3(scene.cam.basis) *
        linalg.matrix4_scale([3]f32{tan_fov_x, tan_fov_y, 1}) *
        linalg.matrix4_translate([3]f32{-1, -1, 1}) *
        linalg.matrix4_scale(1 / [3]f32{dims_f32.x / 2, dims_f32.y / 2, 1})

    timings := make([]time.Duration, number_of_trials)
    defer delete(timings)

    for trial in 0..<number_of_trials {
        start_instant := time.now()

        for sample in 0..<rc.samples {
            for px in 0..<rc.dims.x {
                for py in 0..<rc.dims.y {
                    raw_pixel := [4]f32{
                        cast(f32)px + rand.float32(),
                        cast(f32)py + rand.float32(),
                        0.0, 1.0
                    }

                    direction := linalg.normalize((pixel_to_ray_dir * raw_pixel).xyz)

                    debug_info = {
                        rc = rc,
                        pixel = {px, py},
                    }

                    exitance := raytrace(scene, Ray{scene.cam.pos, direction}, rc.ray_depth)

                    rc_set_pixel(rc, {px, py}, exitance, 0)

                    if is_interrupted() do return
                }
            }
        }

        end_instant := time.now()
        duration := time.diff(start_instant, end_instant)
        timings[trial] = duration
        fmt.printfln("Trial %v >>> Rendered in %v", trial, duration)
    }

    slice.sort(timings)
    total_time := 0.0
    total_time_squared := 0.0
    for duration in timings {
        seconds := time.duration_seconds(duration)
        total_time += seconds
        total_time_squared += seconds * seconds
    }
    average_time := total_time / cast(f64)number_of_trials
    average_time_squared := total_time_squared / cast(f64)number_of_trials
    std_dev := math.sqrt(average_time_squared - average_time * average_time)
    if number_of_trials > 1 {
        std_dev *= math.sqrt(cast(f64)number_of_trials / cast(f64)(number_of_trials - 1))
    } else {
        std_dev = math.INF_F64
    }

    if number_of_trials > 1 {
        fmt.printfln(">>>>>>>>> Performance Summary <<<<<<<<<")
        fmt.printfln("Trials: %v", number_of_trials)
        fmt.printfln("Time: %.02f±%0.02fms",
            average_time * 1000,
            std_dev * 1000,
        )
        fmt.printfln("Best: %.02fms, Median: %.02fms, Worst: %.02fms",
            time.duration_milliseconds(timings[0]),
            time.duration_milliseconds((
                timings[len(timings) / 2] +
                timings[(len(timings) + 1) / 2] \
            ) / 2),
            time.duration_milliseconds(timings[len(timings) - 1]),
        )
        fmt.printfln(">>>>>>>>> Performance Summary <<<<<<<<<")
    }
}

