package raytracer

import "core:math"
import "core:math/linalg"

sq :: #force_inline proc(x: $T) -> T {
    return x * x
}

norm_l1 :: #force_inline proc(x: [3]$T) -> T {
    return compsum(linalg.abs(x))
}

compsum :: #force_inline proc(x: [3]$T) -> T {
    return x.x + x.y + x.z
}

ceil_div :: #force_inline proc(x, y: $T) -> T {
    return (x + y - 1) / y
}

world_to_screen :: proc(rc: Rc, cam: ^Cam, point: [3]f32) -> [2]f32 {
    p := point - cam.pos
    p = linalg.inverse(cam.basis) * p
    if abs(p.z) < 1e-6 {
        return {math.nan_f32(), math.nan_f32()}
    }
    p /= p.z
    dims_f32 := linalg.to_f32(rc.dims)
    aspect_ratio := dims_f32.x / dims_f32.y
    tan_fov_x := math.tan(cam.fov_x / 2)
    tan_fov_y := tan_fov_x / aspect_ratio
    screen01 := p.xy / [2]f32{tan_fov_x, tan_fov_y} * 0.5 + [2]f32{0.5, 0.5}
    screen := screen01 * dims_f32
    screen.y = dims_f32.y - screen.y
    return screen
}

// ChatGPT-ed code!!!
line_to_screen :: proc(rc: Rc, cam: ^Cam, p0_world, p1_world: [3]f32) -> (s1, s2: [2]f32, ok: bool) {
    // Transform endpoints to camera space.
    p0 := p0_world - cam.pos
    p1 := p1_world - cam.pos
    p0 = linalg.inverse(cam.basis) * p0
    p1 = linalg.inverse(cam.basis) * p1

    // Define near threshold and compute frustum tangents.
    dims_f32 := linalg.to_f32(rc.dims)
    aspect_ratio := dims_f32.x / dims_f32.y
    tan_fov_x := math.tan(cam.fov_x / 2)
    tan_fov_y := tan_fov_x / aspect_ratio

    // A helper procedure to clip the line against one plane.
    // The plane is defined by a function f(p) such that f(p) >= 0 means "inside".
    clip :: proc($plane: proc([3]f32, $T) -> f32, x: T, p0_ptr, p1_ptr: ^[3]f32) -> bool {
        f0 := plane(p0_ptr^, x)
        f1 := plane(p1_ptr^, x)
        // If both endpoints are outside, reject the segment.
        if f0 < 0 && f1 < 0 do return false
        // If one endpoint is outside, compute intersection.
        if f0 < 0 {
            // Solve: f(p0 + t*(p1-p0)) = 0  ->  t = f0/(f0 - f1)
            t := f0 / (f0 - f1)
            p0_ptr^ = p0_ptr^ + (p1_ptr^ - p0_ptr^) * t
        } else if f1 < 0 {
            t := f0 / (f0 - f1)
            p1_ptr^ = p0_ptr^ + (p1_ptr^ - p0_ptr^) * t
        }
        return true
    }

    // Define the five view–frustum planes in camera space:
    // (For each plane, the condition f(p) >= 0 means "inside".)
    near_plane :: proc(point: [3]f32, _: u8) -> f32 { return point.z - 1e-3 }
    left_plane :: proc(point: [3]f32, tan_fov_x: f32) -> f32 { return point.x + tan_fov_x * point.z }
    right_plane :: proc(point: [3]f32, tan_fov_x: f32) -> f32 { return tan_fov_x * point.z - point.x }
    bottom_plane :: proc(point: [3]f32, tan_fov_y: f32) -> f32 { return point.y + tan_fov_y * point.z }
    top_plane :: proc(point: [3]f32, tan_fov_y: f32) -> f32 { return tan_fov_y * point.z - point.y }
    if (!clip(near_plane, 0, &p0, &p1) ||
        !clip(left_plane, tan_fov_x, &p0, &p1) ||
        !clip(right_plane, tan_fov_x, &p0, &p1) ||
        !clip(bottom_plane, tan_fov_y, &p0, &p1) ||
        !clip(top_plane, tan_fov_y, &p0, &p1)) {
        ok = false
        return
    }

    // Helper to project a clipped camera–space point to screen space.
    p0 /= p0.z
    p1 /= p1.z
    screen01_0 := p0.xy / [2]f32{tan_fov_x, tan_fov_y} * 0.5 + [2]f32{0.5, 0.5}
    screen01_1 := p1.xy / [2]f32{tan_fov_x, tan_fov_y} * 0.5 + [2]f32{0.5, 0.5}
    screen0 := screen01_0 * dims_f32
    screen1 := screen01_1 * dims_f32
    screen0.y = dims_f32.y - screen0.y // Flip y.
    screen1.y = dims_f32.y - screen1.y // Flip y.
    return screen0, screen1, true
}
