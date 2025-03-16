package raytracer

import "core:fmt"
import "core:sync"
import "core:math"
import "core:slice"
import "core:testing"
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
    material_kind: enum u32 {
        Diffuse, Metallic, Dielectric,
    },
    ior: f32,
}

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

Cam :: struct {
    pos: [3]f32,
    basis: matrix[3, 3]f32,
    fov_x: f32,
}

Scene :: struct {
    cam: Cam,
    ambient: [3]f32,
    lights: [dynamic]Light,
    objects: [dynamic]Object,
}

destory_scene :: proc(s: Scene) {
    delete(s.objects)
}

Ray :: struct {
    o, d: [3]f32,
}

// If t is negative, the ray does *not* intersect the geometry,
// and other fields have arbitrary values
Geometry_Hit :: struct {
    inside: bool,
    n: [3]f32,
    t: f32,
}

intersect_ray_plane :: proc(ray: Ray, plane: Plane) -> (hit: Geometry_Hit) {
    using hit
    n = plane.normal
    o_dot_n := dot(ray.o, n)
    t = -o_dot_n / dot(ray.d, n)
    inside = o_dot_n < 0
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
    inside = t1 < 0
    t = t2 if inside else t1
    p := ray.o + t * ray.d
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
    p := ray.o + t * ray.d
    n = linalg.step(box.extent, linalg.abs(p) + 1e-6) * linalg.sign(p)
    return
}

Hit :: struct {
    object_id: int,
    t: f32,
    p, n: [3]f32,
    color: [3]f32,
}

cast_ray :: proc(scene: Scene, ray: Ray, max_dist: f32) -> (hit: Hit) {
    hit.object_id = 0
    hit.t = max_dist

    for object, i in scene.objects {
        if i == 0 do continue

        conj_rotation := conj(object.rotation)
        local_ray := Ray{
            o = linalg.mul(conj_rotation, ray.o - object.pos),
            d = linalg.mul(conj_rotation, ray.d),
        }

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
                object_id = i,
                // p and color are set later
            }
        }
    }

    hit.p = ray.o + hit.t * ray.d

    return
}

raytrace :: proc(scene: Scene, ray: Ray, depth_left: int) -> [3]f32 {
    return scene.objects[0].color
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
        linalg.matrix4_scale(1 / [3]f32{dims_f32.x / 2, dims_f32.y / 2, 1}) *
        linalg.matrix4_translate([3]f32{0.5, 0.5, 0.0})

    timings := make([]time.Duration, number_of_trials)
    defer delete(timings)

    for trial in 0..<number_of_trials {
        start_instant := time.now()

        for px in 0..<rc.dims.x {
            for py in 0..<rc.dims.y {
                raw_pixel := [4]f32{cast(f32)px, cast(f32)py, 0.0, 1.0}

                direction := linalg.normalize((pixel_to_ray_dir * raw_pixel).xyz)

                hit := cast_ray(scene, Ray{scene.cam.pos, direction}, math.INF_F32)

                rc_set_pixel(rc, {px, py}, hit.color)

                if is_interrupted() do return
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

