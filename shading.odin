package raytracer

import "core:math/rand"
import "core:math/linalg"
import "core:math"
import "core:fmt"
import sa "core:container/small_array"

sphere_uniform :: proc() -> [3]f32 {
    phi := rand.float32_range(0.0, math.TAU)
    z := rand.float32_range(-1.0, 1.0)
    x, y := math.sincos(phi)
    radius := math.sqrt(1 - sq(z))
    return {x * radius, y * radius, z}
}

sphere_uniform_pdf :: proc(omega: [3]f32) -> f32 {
    return 1 / (4 * math.PI)
}

halfsphere_uniform :: proc(n: [3]f32) -> [3]f32 {
    v := sphere_uniform()
    if dot(v, n) < 0.0 do v = -v
    return v
}

halfsphere_uniform_pdf :: proc(n: [3]f32, omega: [3]f32) -> f32 {
    if dot(omega, n) < 0.0 do return 0.0
    return 1 / (2 * math.PI)
}

cosine_weighted :: proc(n: [3]f32) -> [3]f32 {
    sphere := sphere_uniform() + n
    return linalg.normalize(sphere)
}

cosine_weighted_pdf :: proc(n: [3]f32, omega: [3]f32) -> f32 {
    return max(dot(n, omega) / math.PI, 0)
}

surface_sampling :: proc(trigs: []Triangle, origin: [3]f32) -> [3]f32 {
    index := rand.int_max(len(trigs))
    trig := trigs[index]
    local: [3]f32
    u := rand.float32_range(0, 1)
    v := rand.float32_range(0, 1)
    if u + v > 1 do u, v = 1 - u, 1 - v
    world := trig.p + u * trig.u + v * trig.v
    return linalg.normalize(world - origin)
}

surface_sampling_pdf_trigs_sum :: proc(trigs: []Triangle, ray: Ray) -> (p: f32) {
    for trig in trigs {
        hit := intersect_ray_triangle(ray, trig)
        if !(hit.t >= 0) do continue
        weight := sq(hit.t) / abs(dot(trig.ng, ray.d))
        p += 2 / linalg.length(linalg.cross(trig.u, trig.v)) * weight
    }
    return p
}

surface_sampling_pdf_bvh_sum :: proc(
    bvh: []BVH_Node, trigs: []Triangle,
    global_ray: Ray,
) -> (p: f32) {
    RAY_EPS :: 1e-3
    ray := Ray{
        d = global_ray.d,
        o = global_ray.o + global_ray.d * RAY_EPS,
    }

    if _, h := check_intersect_ray_aabb(ray, bvh[len(bvh) - 1].aabb, math.INF_F32); !h {
        return
    }

    stack: sa.Small_Array(64, int)
    sa.append(&stack, len(bvh) - 1)

    for sa.len(stack) > 0 {
        id := sa.pop_back(&stack)
        node := bvh[id]
        switch vertex in node.vertex {
        case BVH_Node_Leaf:
            p += surface_sampling_pdf_trigs_sum(vertex.trigs, ray)
        case BVH_Node_Branch:
            _, hl := check_intersect_ray_aabb(ray, bvh[vertex.left].aabb, math.INF_F32)
            _, hr := check_intersect_ray_aabb(ray, bvh[vertex.right].aabb, math.INF_F32)
            if hl do sa.append(&stack, vertex.left)
            if hr do sa.append(&stack, vertex.right)
        }
    }

    return p
}

surface_sampling_pdf :: proc(scene: ^Scene, ray: Ray) -> (p: f32) {
    // objects_pdf := surface_sampling_pdf_objects_sum(scene.light_surfaces[:], ray)
    objects_pdf := surface_sampling_pdf_bvh_sum(scene.light_bvh[:], scene.light_surfaces[:], ray)
    return objects_pdf / f32(len(scene.light_surfaces))
}

vndf_sampling :: proc(n, omega: [3]f32, alpha: f32) -> [3]f32 {
    u1, u2 := rand.float32(), rand.float32()
    w := math.sqrt((1 + n.z) / 2)
    rotation := w > 0 ? quaternion(w = w, x = -n.y / (2 * w), y = n.x / (2 * w), z = 0) :
        quaternion(w = 0, x = 1, y = 0, z = 0)
    V := linalg.mul(conj(rotation), omega)
    Vh := linalg.normalize([3]f32{alpha * V.x, alpha * V.y, V.z})
    len := math.hypot(Vh.x, Vh.y)
    T1 := len == 0 ? [3]f32{1, 0, 0} : [3]f32{-Vh.y / len, Vh.x / len, 0}
    T2 := linalg.cross(Vh, T1)
    r := math.sqrt(u1)
    phi := math.TAU * u2
    t1, t2 := math.sincos(phi)
    t1 *= r
    t2 *= r
    s := 0.5 * (1 + Vh.z)
    t2 = (1 - s) * math.sqrt(1 - sq(t1)) + s * t2
    Nh := t1 * T1 + t2 * T2 + Vh * math.sqrt(max(0, 1 - sq(t1) - sq(t2)))
    Ne := linalg.normalize([3]f32{alpha * Nh.x, alpha * Nh.y, max(0, Nh.z)})
    return linalg.mul(rotation, Ne)
}

vndf_sampling_pdf :: proc(n, omega: [3]f32, alpha: f32, L: [3]f32) -> f32 {
    Ne := linalg.normalize(omega + L)
    w := math.sqrt((1 + n.z) / 2)
    rotation := w > 0 ? quaternion(w = w, x = -n.y / (2 * w), y = n.x / (2 * w), z = 0) :
        quaternion(w = 0, x = 1, y = 0, z = 0)
    V := linalg.mul(conj(rotation), omega)
    N := linalg.mul(conj(rotation), Ne)
    alpha2 := sq(alpha)
    lambda := (-1 + math.sqrt(1 + alpha2 * (sq(V.x) + sq(V.y)) / sq(V.z))) * 0.5
    G1 := 1 / (1 + lambda)
    D := 1 / (math.PI * alpha2 * sq(sq(N.x / alpha) + sq(N.y / alpha) + sq(N.z)))
    normal := G1 * max(0, dot(V, N)) * D / V.z
    return normal / (4 * dot(L, Ne))
}

sample :: proc(
    scene: ^Scene, mat: Point_Material, in_ray: Ray,
) -> (d: [3]f32) {
    t := rand.float32()
    if t <= 0.33333 {
        return cosine_weighted(mat.normal)
    } else if t < 0.666666 && len(scene.light_surfaces) > 0 {
        return surface_sampling(scene.light_surfaces[:], mat.pos)
    } else {
        n := vndf_sampling(mat.normal, -in_ray.d, sq(mat.roughness))
        return in_ray.d - 2 * dot(n, in_ray.d) * n
    }
}

pdf :: proc(
    scene: ^Scene, mat: Point_Material,
    in_ray: Ray, out_ray: Ray,
) -> (p: f32) {
    has_lights := len(scene.light_surfaces) > 0
    return (cosine_weighted_pdf(mat.normal, out_ray.d) +
        (surface_sampling_pdf(scene, out_ray) if has_lights else 0) +
        vndf_sampling_pdf(mat.normal, -in_ray.d, sq(mat.roughness), out_ray.d) *
            (1 if has_lights else 2)) / 3
}

shade :: proc(
    scene: ^Scene, mat: Point_Material,
    in_ray: Ray, out_ray: Ray
) -> (exitance: [3]f32) {
    alpha := sq(mat.roughness)
    alpha2 := sq(alpha)

    L := out_ray.d
    V := -in_ray.d
    H := linalg.normalize(L + V)
    N := mat.normal

    cosine := dot(L, N)

    f0, f90 :: 0.04, 1
    frensel_base := math.pow(1 - dot(H, L), 5)
    frensel_diff_spec := f0 + (f90 - f0) * frensel_base
    frensel_metallic := mat.color + (f90 - mat.color) * frensel_base

    distribution_term :=
        alpha2 * math.step(f32(0), dot(H, N)) /
            (math.PI * sq((alpha2 - 1) * sq(dot(H, N)) + 1))

    smith_geometry_ggx :: proc(n, x: [3]f32, alpha2: f32) -> f32 {
        cosine := dot(n, x)
        return 2 * max(cosine, 0) / (cosine + math.sqrt(alpha2 + (1 - alpha2) * sq(cosine)))
    }

    geometry_term := smith_geometry_ggx(N, L, alpha2) * smith_geometry_ggx(N, V, alpha2)

    cook_torrance := distribution_term * geometry_term / (4 * dot(V, N))
    specular := cook_torrance * [3]f32{1, 1, 1}

    diffuse := mat.color * max(cosine, 0) / math.PI

    metallic := specular * frensel_metallic
    dielectic := math.lerp(diffuse, specular, frensel_diff_spec)

    result := math.lerp(dielectic, metallic, mat.metallic)
    return result
}

