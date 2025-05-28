package raytracer

import "core:math/rand"
import "core:math/linalg"
import "core:math"
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
    if linalg.dot(v, n) < 0.0 do v = -v
    return v
}

halfsphere_uniform_pdf :: proc(n: [3]f32, omega: [3]f32) -> f32 {
    if linalg.dot(omega, n) < 0.0 do return 0.0
    return 1 / (2 * math.PI)
}

cosine_weighted :: proc(n: [3]f32) -> [3]f32 {
    sphere := sphere_uniform() + n
    return linalg.normalize(sphere)
}

cosine_weighted_pdf :: proc(n: [3]f32, omega: [3]f32) -> f32 {
    return max(linalg.dot(n, omega) / math.PI, 0)
}

surface_sampling :: proc(objects: []Object, origin: [3]f32) -> [3]f32 {
    object_index := rand.int_max(len(objects))
    object := objects[object_index]
    local: [3]f32
    u := rand.float32_range(0, 1)
    v := rand.float32_range(0, 1)
    if u + v > 1 do u, v = 1 - u, 1 - v
    world := object.trig.p + u * object.trig.u + v * object.trig.v
    return linalg.normalize(world - origin)
}

surface_sampling_pdf_objects_sum :: proc(objects: []Object, ray: Ray) -> (p: f32) {
    for object in objects {
        hit := intersect_ray_triangle(ray, object.trig)
        if !(hit.t >= 0) do continue
        xy := ray.o - hit.p
        weight := linalg.length2(xy) / abs(linalg.dot(hit.ng, ray.d))
        p += 2 / linalg.length(linalg.cross(object.trig.u, object.trig.v)) * weight
    }
    return p
}

surface_sampling_pdf_bvh_sum :: proc(
    bvh: []BVH_Node, objects: []Object,
    ray: Ray,
) -> (p: f32) {
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
            p += surface_sampling_pdf_objects_sum(vertex.objects, ray)
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

sample :: proc(
    scene: ^Scene, mat: Material,
    in_ray: Ray, hit: Hit
) -> (d: [3]f32) {
    switch mat.material_kind {
    case .Diffuse:
        cosine_weighted_weight: f32 = len(scene.light_surfaces) > 0 ? 0.5 : 1.0
        if rand.float32() <= cosine_weighted_weight {
            return cosine_weighted(hit.ns)
        } else {
            return surface_sampling(scene.light_surfaces[:], hit.p)
        }
    case .Metallic:
        return in_ray.d - 2 * linalg.dot(hit.ns, in_ray.d) * hit.ns
    case .Dielectric:
        rel_ior := mat.ior if hit.inside else 1 / mat.ior
        cos_theta1 := -dot(hit.ns, in_ray.d)
        sin_theta2 := math.sqrt(1 - sq(cos_theta1)) * rel_ior

        if sin_theta2 < 1 {
            base_reflection := sq((rel_ior - 1) / (rel_ior + 1))
            ratio := base_reflection + (1 - base_reflection) * math.pow(1 - cos_theta1, 5)

            if rand.float32() > ratio {
                cos_theta2 := math.sqrt(1 - sq(sin_theta2))
                return linalg.normalize(in_ray.d * rel_ior + hit.ns * (rel_ior * cos_theta1 - cos_theta2))
            } else {
                return in_ray.d + 2 * cos_theta1 * hit.ns
            }
        } else {
            d_reflected := in_ray.d + 2 * cos_theta1 * hit.ns
            return d_reflected
        }
    }
    unreachable()
}

pdf :: proc(
    scene: ^Scene, mat: Material,
    in_ray: Ray, hit: Hit, out_ray: Ray,
) -> (p: f32) {
    switch mat.material_kind {
    case .Diffuse:
        cosine_weighted_weight: f32 = len(scene.light_surfaces) > 0 ? 0.5 : 1.0
        return len(scene.light_surfaces) > 0 ? math.lerp(
            surface_sampling_pdf(scene, out_ray),
            cosine_weighted_pdf(hit.ns, out_ray.d),
            cosine_weighted_weight,
        ) : cosine_weighted_pdf(hit.ns, out_ray.d)
    case .Metallic:
        return 1
    case .Dielectric:
        return 1
    }
    unreachable()
}

shade :: proc(
    scene: ^Scene, mat: Material,
    in_ray: Ray, hit: Hit, out_ray: Ray
) -> (exitance: [3]f32) {
    switch mat.material_kind {
    case .Diffuse:
        cosine := linalg.dot(out_ray.d, hit.ns)
        return mat.color / math.PI * max(cosine, 0)
    case .Metallic:
        return mat.color
    case .Dielectric:
        if hit.inside || linalg.dot(out_ray.d, hit.ng) < 0 {
            return 1
        }
        return mat.color
    }
    unreachable()
}

