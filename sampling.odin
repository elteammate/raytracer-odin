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
    switch geometry in object.geometry {
    case Plane:
        panic("Cannot sample a point on a plane")
    case Ellipsoid:
        local = sphere_uniform() * geometry.radii
    case Box:
        areas := linalg.abs(geometry.extent.yzx * geometry.extent.zxy)
        w := rand.float32_range(0, norm_l1(areas) * 2)
        x := rand.float32_range(-1, 1)
        y := rand.float32_range(-1, 1)
        if w < areas.x {
            local = geometry.extent * {1, x, y}
        } else if w < 2 * areas.x {
            local = geometry.extent * {-1, x, y}
        } else if w < 2 * areas.x + areas.y {
            local = geometry.extent * {x, 1, y}
        } else if w < 2 * areas.x + 2 * areas.y {
            local = geometry.extent * {x, -1, y}
        } else if w < 2 * areas.x + 2 * areas.y + areas.z {
            local = geometry.extent * {x, y, 1}
        } else {
            local = geometry.extent * {x, y, -1}
        }
    case Triangle:
        u := rand.float32_range(0, 1)
        v := rand.float32_range(0, 1)
        if u + v > 1 do u, v = 1 - u, 1 - v
        local = geometry.p + u * geometry.u + v * geometry.v
    }
    world := object.pos + linalg.mul(object.rotation, local)
    return linalg.normalize(world - origin)
}

surface_sampling_pdf_objects_sum :: proc(objects: []Object, ray: Ray) -> (p: f32) {
    for object in objects {
        conj_rotation := conj(object.rotation)
        local_d := linalg.mul(conj_rotation, ray.d)
        local_o := linalg.mul(conj_rotation, ray.o + 1e-3 * ray.d - object.pos)
        local_ray := Ray{o = local_o, d = local_d}

        switch geometry in object.geometry {
        case Plane:
            panic("Cannot sample a point on a plane")
        case Ellipsoid:
            hit := intersect_ray_ellipsoid(local_ray, geometry)
            if !(hit.t >= 0) do continue
            xy := local_o - hit.p
            weight := linalg.length2(xy) / abs(linalg.dot(hit.n, local_d))
            p += 1 / (4 * math.PI * linalg.length(hit.p / geometry.radii * geometry.radii.yzx * geometry.radii.zxy)) * weight
            local_ray.o = hit.p + local_d * 1e-4
            hit = intersect_ray_ellipsoid(local_ray, geometry)
            if !(hit.t >= 0) do continue
            xy = local_o - hit.p
            weight = linalg.length2(xy) / abs(linalg.dot(hit.n, local_d))
            p += 1 / (4 * math.PI * linalg.length(hit.p / geometry.radii * geometry.radii.yzx * geometry.radii.zxy)) * weight
        case Box:
            hit := intersect_ray_box(local_ray, geometry)
            if !(hit.t >= 0) do continue
            xy := local_o - hit.p
            weight := linalg.length2(xy) / abs(linalg.dot(hit.n, local_d))
            p += 0.125 / norm_l1(geometry.extent.xyz * geometry.extent.yzx) * weight
            local_ray.o = hit.p + local_d * 1e-4
            hit = intersect_ray_box(local_ray, geometry)
            if !(hit.t >= 0) do continue
            xy = local_o - hit.p
            weight = linalg.length2(xy) / abs(linalg.dot(hit.n, local_d))
            p += 0.125 / norm_l1(geometry.extent.xyz * geometry.extent.yzx) * weight
        case Triangle:
            hit := intersect_ray_triangle(local_ray, geometry)
            if !(hit.t >= 0) do continue
            xy := local_o - hit.p
            weight := linalg.length2(xy) / abs(linalg.dot(hit.n, local_d))
            p += 2 / linalg.length(linalg.cross(geometry.u, geometry.v)) * weight
        }
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
    objects_pdf := surface_sampling_pdf_objects_sum(scene.standalone_lights[:], ray)
    objects_pdf += surface_sampling_pdf_bvh_sum(scene.light_bvh[:], scene.light_surfaces[:], ray)
    return objects_pdf / f32(len(scene.light_surfaces))
}

