package raytracer

import "core:fmt"
import "core:sync"
import "core:math"
import "core:math/linalg"
import "core:time"

Plane :: struct { normal: [3]f32 }
Ellipsoid :: struct { radii: [3]f32 }
Box :: struct { extent: [3]f32 }

Geometry :: union { Plane, Ellipsoid, Box }

Object :: struct {
    geometry: Geometry,
    pos: [3]f32,
    rotation: quaternion128,
    color: [3]f32,
}

Cam :: struct {
    pos: [3]f32,
    basis: matrix[3, 3]f32,
    fov_x: f32,
}

Scene :: struct {
    bg_color: [3]f32,
    cam: Cam,
    objects: [dynamic]Object,
}

destory_scene :: proc(s: Scene) {
    delete(s.objects)
}

Ray :: struct {
    o, d: [3]f32,
}

render_scene :: proc(rc: Rc, scene: Scene) {
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

    for px in 0..<rc.dims.x {
        for py in 0..<rc.dims.y {
            raw_pixel := [4]f32{cast(f32)px, cast(f32)py, 0.0, 1.0}

            direction := linalg.normalize((pixel_to_ray_dir * raw_pixel).xyz)

            rc_set_pixel(rc, {px, py}, {1.0, 1.0, 0.5})

            if is_interrupted() do return
        }
    }
}

