package raytracer

import "core:math/rand"
import "core:math/linalg"
import "core:math"

sphere_uniform :: proc() -> [3]f32 {
    phi := rand.float32_range(0.0, math.TAU)
    z := rand.float32_range(-1.0, 1.0)
    x, y := math.sincos(phi)
    radius := math.sqrt(1 - sq(z))
    return {x * radius, y * radius, z}
}

halfsphere_uniform :: proc(n: [3]f32) -> [3]f32 {
    v := sphere_uniform()
    if linalg.dot(v, n) < 0.0 do v = -v
    return v
}

