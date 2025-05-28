package raytracer

import "base:runtime"
import "core:fmt"
import "core:sync"
import "core:math"
import "core:sort"
import sa "core:container/small_array"
import "core:slice"
import "core:testing"
import "core:math/rand"
import "core:math/linalg"
import "core:time"
import "core:thread"

dot :: linalg.dot

Triangle :: struct {
    p, u, v, n1, n2, n3, ng: [3]f32
}

Material_Kind :: enum u32 {
    Diffuse, Metallic, Dielectric,
}

Object :: struct {
    trig: Triangle,
    material: Material,
}

Material :: struct {
    material_kind: Material_Kind,
    color: [3]f32,
    emission: [3]f32,
    metallic: f32,
    roughness: f32,
    ior: f32,
}

Cam :: struct {
    pos: [3]f32,
    basis: matrix[3, 3]f32,
    fov_x: f32,
}

Scene :: struct {
    cam: Cam,
    objects: [dynamic]Object,
    light_surfaces: [dynamic]Object,
    bvh: [dynamic]BVH_Node,
    light_bvh: [dynamic]BVH_Node,
}

finish_scene :: proc(rc: Rc, s: ^Scene) {
    for object in s.objects[1:] {
        if norm_l1(object.material.emission) > 1e-6 {
            append(&s.light_surfaces, object)
        }

        rc_log_aabb(rc, aabb_of_object(object), object.material.color)
    }

    now := time.now()
    s.bvh = bvh_build(s.objects[1:])
    fmt.printfln("Scene BVH built in %v", time.diff(now, time.now()))
    now = time.now()
    s.light_bvh = bvh_build(s.light_surfaces[:])
    fmt.printfln("Light BVH built in %v", time.diff(now, time.now()))

    traverse_bvh_add_aabbs :: proc(rc: Rc, nodes: []BVH_Node, id: int, level: u32) {
        node := nodes[id]
        switch vertex in node.vertex {
        case BVH_Node_Leaf:
            rc_log_aabb(rc, node.aabb, tag = level)
        case BVH_Node_Branch:
            rc_log_aabb(rc, node.aabb, tag = level)
            traverse_bvh_add_aabbs(rc, nodes, vertex.left, level + 1)
            traverse_bvh_add_aabbs(rc, nodes, vertex.right, level + 1)
        }
    }

    traverse_bvh_add_aabbs(rc, s.bvh[:], len(s.bvh) - 1, 1)

}

destory_scene :: proc(s: ^Scene) {
    delete(s.objects)
    delete(s.light_surfaces)
    delete(s.bvh)
    delete(s.light_bvh)
}

Ray :: struct {
    o, d: [3]f32,
}

// If t is negative, the ray does *not* intersect the geometry,
// and other fields have arbitrary values
Geometry_Hit :: struct {
    // whether the ray is inside the geometry
    inside: bool,
    // geometry normal, shading normal, hit position
    ng, ns, p: [3]f32,
    t: f32,
}

check_intersect_ray_aabb :: proc(global_ray: Ray, aabb: AABB, max_dist: f32) -> (f32, bool) {
    ray := Ray{o = global_ray.o - aabb.lo, d = global_ray.d}
    extent := aabb.hi - aabb.lo
    if linalg.length(ray.o - extent / 2) - linalg.length(extent / 2) > max_dist {
        return 0, false
    }
    t1_raw := (extent - ray.o) / ray.d
    t2_raw := -ray.o / ray.d
    t_min := linalg.min(t1_raw, t2_raw)
    t_max := linalg.max(t1_raw, t2_raw)
    t1 := max(t_min.x, t_min.y, t_min.z)
    t2 := min(t_max.x, t_max.y, t_max.z)
    if t1 > t2 do return 0, false
    if t2 < 0 do return 0, false
    return max(t1, 0), true
}

intersect_ray_triangle :: proc(ray: Ray, trig: Triangle) -> (hit: Geometry_Hit) {
    b := ray.o - trig.p
    a: matrix[3, 3]f32
    a[0] = trig.u
    a[1] = trig.v
    a[2] = -ray.d
    u, v, t := expand_values(linalg.inverse(a) * b)
    if u < 0 || v < 0 || u + v > 1 do return Geometry_Hit{t = -1}
    return Geometry_Hit{
        t = t,
        p = ray.o + t * ray.d,
        ng = trig.ng,
        ns = linalg.normalize(trig.n1 * (1 - u - v) + trig.n2 * u + trig.n3 * v),
        inside = dot(trig.ng, ray.d) > 0,
    }
}

AABB :: struct {
    lo, hi: [3]f32
}

AABB_EMPTY :: AABB{
    lo = {math.INF_F32, math.INF_F32, math.INF_F32},
    hi = {-math.INF_F32, -math.INF_F32, -math.INF_F32},
}

aabb_merge :: proc(lhs, rhs: AABB) -> AABB {
    return AABB {
        lo = linalg.min(lhs.lo, rhs.lo),
        hi = linalg.max(lhs.hi, rhs.hi),
    }
}

aabb_shift :: proc(aabb: AABB, offset: [3]f32) -> AABB {
    return AABB {
        lo = aabb.lo + offset,
        hi = aabb.hi + offset,
    }
}

box_corners :: proc(lo, hi: [3]f32) -> [8][3]f32 {
    return {
        {lo.x, lo.y, lo.z},
        {lo.x, lo.y, hi.z},
        {lo.x, hi.y, lo.z},
        {lo.x, hi.y, hi.z},
        {hi.x, lo.y, lo.z},
        {hi.x, lo.y, hi.z},
        {hi.x, hi.y, lo.z},
        {hi.x, hi.y, hi.z},
    }
}

aabb_of_points :: proc(points: [$N][3]f32) -> AABB {
    aabb := AABB{lo = points[0], hi = points[0]}
    for point in points {
        aabb.lo = linalg.min(aabb.lo, point)
        aabb.hi = linalg.max(aabb.hi, point)
    }
    return aabb
}

aabb_of_object :: proc(object: Object) -> AABB {
    trig := object.trig
    points := [3][3]f32{
        trig.p,
        trig.p + trig.u,
        trig.p + trig.v,
    }
    return aabb_of_points(points)
}

aabb_area :: proc(aabb: AABB) -> f32 {
    size := aabb.hi - aabb.lo
    return compsum(size.xyz * size.yzx)
}

BVH_Node_Leaf :: struct {
    objects: []Object,
}

BVH_Node_Branch :: struct {
    left, right: int,
}

BVH_Node :: struct {
    aabb: AABB,
    vertex: union #no_nil {
        BVH_Node_Leaf,
        BVH_Node_Branch,
    },
}

bvh_build :: proc(objects: []Object) -> [dynamic]BVH_Node {
    objects := objects

    LEAF_NODE_THRESHOLD :: 4

    recurse :: proc(
        buf: []AABB,
        aabbs: []AABB,
        objects: []Object,
        nodes: ^[dynamic]BVH_Node,
    ) -> (node_index: int) {
        Sorter :: struct {
            aabbs: []AABB,
            objects: []Object,
        }

        if len(objects) <= LEAF_NODE_THRESHOLD {
            aabb := AABB_EMPTY
            for i in 0..<len(objects) {
                aabb = aabb_merge(aabb, aabbs[i])
            }
            node := BVH_Node{
                aabb = aabb,
                vertex = BVH_Node_Leaf{objects = objects}
            }
            append(nodes, node)
            return len(nodes) - 1
        }

        sorter_interface :: proc($axis: uint) -> sort.Interface where axis < 3 {
            return sort.Interface{
                len = proc(iface: sort.Interface) -> int {
                    return len((cast(^Sorter)(iface.collection)).aabbs)
                },
                less = proc(iface: sort.Interface, i, j: int) -> bool {
                    aabbs := (cast(^Sorter)(iface.collection)).aabbs
                    return aabbs[i].lo[axis] < aabbs[j].lo[axis]
                },
                swap = proc(iface: sort.Interface, i, j: int) {
                    sorter := cast(^Sorter)(iface.collection)
                    slice.swap(sorter.aabbs, i, j)
                    slice.swap(sorter.objects, i, j)
                }
            }
        }

        best_value: f32 = math.INF_F32
        best_axis: int = 0

        try_axis :: proc(
            $axis: uint, objects: []Object,
            buf: []AABB, aabbs: []AABB
        ) -> (f32, int) {
            best_sah: f32 = math.INF_F32
            best_index: int = 0
            interface := sorter_interface(axis)
            sorter := Sorter{
                aabbs = aabbs,
                objects = objects,
            }
            interface.collection = &sorter
            #force_inline sort.sort(interface)
            for i := len(objects) - 1; i >= 0; i -= 1 {
                buf[i] = aabbs[i]
                if i != len(objects) - 1 {
                    buf[i] = aabb_merge(buf[i], buf[i + 1])
                }
            }
            rolling_area: f32 = 0
            aabb_total := AABB_EMPTY
            for i in 1..<len(objects) {
                aabb_total = aabb_merge(aabb_total, aabbs[i - 1])
                sah := aabb_area(aabb_total) * f32(i) +
                    aabb_area(buf[i]) * f32(len(objects) - i)
                if sah < best_sah do best_sah, best_index = sah, i
            }
            return best_sah, best_index
        }

        sah0, _ := try_axis(0, objects, buf, aabbs)
        aabb_total := buf[0]
        sah1, _ := try_axis(1, objects, buf, aabbs)
        sah2, _ := try_axis(2, objects, buf, aabbs)
        split: int
        if sah0 < sah1 && sah0 < sah2 {
            _, split = try_axis(0, objects, buf, aabbs)
        } else if sah1 < sah0 && sah1 < sah2 {
            _, split = try_axis(1, objects, buf, aabbs)
        } else {
            _, split = try_axis(2, objects, buf, aabbs)
        }
        left := recurse(buf, aabbs[:split], objects[:split], nodes)
        right := recurse(buf, aabbs[split:], objects[split:], nodes)
        append(nodes, BVH_Node{
            aabb = aabb_total,
            vertex = BVH_Node_Branch{
                left = left,
                right = right,
            }
        })
        return len(nodes) - 1
    }

    aabbs := make([]AABB, len(objects), context.temp_allocator)
    for i := 0; i < len(objects); i += 1 {
        aabbs[i] = aabb_of_object(objects[i])
    }

    nodes := make([dynamic]BVH_Node, 0, len(objects) / 2)
    buf := make([]AABB, len(objects), context.temp_allocator)
    defer delete(buf, context.temp_allocator)
    defer delete(aabbs, context.temp_allocator)

    recurse(buf, aabbs[:len(objects)], objects, &nodes)
    return nodes
}

Hit :: struct {
    object: ^Object,
    t: f32,
    p, ng, ns: [3]f32,
    inside: bool,
}

cast_ray_through_objects :: proc(objects: []Object, ray: Ray, max_dist: f32) -> (hit: Hit) {
    hit.object = nil
    hit.t = max_dist

    // assert(abs(linalg.length2(ray.d) - 1) < 1e-4)

    for &object, i in objects {
        gh := intersect_ray_triangle(ray, object.trig)

        if gh.t > 0 && gh.t < hit.t {
            hit = {
                t = gh.t, ng = gh.ng, ns = gh.ns,
                object = &objects[i], inside = gh.inside,
                // p is set later
            }
        }
    }

    hit.p = ray.o + hit.t * ray.d

    return
}

cast_ray_through_bvh :: proc(bvh: []BVH_Node, ray: Ray, max_dist: f32) -> (hit: Hit) {
    max_dist := max_dist

    hit.t = max_dist
    if _, h := check_intersect_ray_aabb(ray, bvh[len(bvh) - 1].aabb, max_dist); !h {
        return
    }

    stack: sa.Small_Array(64, int)
    sa.append(&stack, len(bvh) - 1)

    for sa.len(stack) > 0 {
        id := sa.pop_back(&stack)
        node := bvh[id]
        switch vertex in node.vertex {
        case BVH_Node_Leaf:
            cur_hit := cast_ray_through_objects(vertex.objects, ray, max_dist)
            if cur_hit.t < max_dist {
                hit = cur_hit
                max_dist = cur_hit.t
            }
        case BVH_Node_Branch:
            tl, hl := check_intersect_ray_aabb(ray, bvh[vertex.left].aabb, max_dist)
            tr, hr := check_intersect_ray_aabb(ray, bvh[vertex.right].aabb, max_dist)
            if !hl && !hr do continue
            if hl && hr {
                if tl < tr {
                    sa.append(&stack, vertex.left)
                    sa.append(&stack, vertex.right)
                } else {
                    sa.append(&stack, vertex.right)
                    sa.append(&stack, vertex.left)
                }
            }
            if hl {
                sa.append(&stack, vertex.left)
            } else if hr {
                sa.append(&stack, vertex.right)
            }
        }
    }

    return hit
}

cast_ray :: proc(scene: ^Scene, global_ray: Ray, max_dist: f32) -> (hit: Hit) {
    max_dist := max_dist
    RAY_EPS :: 1e-3
    ray := Ray{
        d = global_ray.d,
        o = global_ray.o + global_ray.d * RAY_EPS,
    }

    hit2 := cast_ray_through_bvh(scene.bvh[:], ray, max_dist)
    if hit2.t < max_dist {
        hit = hit2
    }
    hit.t += RAY_EPS
    return hit
}

raytrace :: proc(scene: ^Scene, ray: Ray, depth_left: i32) -> (exitance: [3]f32) {
    if depth_left == 0 do return 0

    hit := cast_ray(scene, ray, math.INF_F32)

    if hit.inside {
        hit.ng = -hit.ng
        hit.ns = -hit.ns
    }

    if hit.object == nil {
        return scene.objects[0].material.color
    }

    object := hit.object^

    d_reflected := sample(scene, object.material, ray, hit)
    reflected := Ray{ o = hit.p, d = d_reflected }
    pdf := pdf(scene, object.material, ray, hit, reflected)
    value := shade(scene, object.material, ray, hit, reflected)
    debug_rc_set(d_reflected, 1)
    debug_rc_set(value / pdf * 1e-3, 2)

    irradiance: [3]f32 = 0
    if norm_l1(value) / pdf > 1e-5 {
        irradiance = raytrace(scene, reflected, depth_left - 1)
        exitance = value * irradiance / pdf + object.material.emission
    } else {
        exitance = object.material.emission
    }

    if norm_l1(exitance) > 1e3 {
        debug_log_ray({
            ray = ray,
            t = hit.t,
            color = {1, 0, 0}
        })
    }
    if norm_l1(value) / pdf > 1e3 {
        debug_log_ray({
            ray = ray,
            t = hit.t,
            color = {0, 1, 0}
        })
    }

    return exitance
}

@(thread_local) debug_info: struct {
    rc: Rc,
    pixel: [2]u32,
}

RENDERING_TILE_SIZE :: 4
RENDERING_TILE_SAMPLES :: 32

render_task :: proc(rc: Rc, scene: ^Scene, tile_id: ^u64) {
    dims_f32 := linalg.to_f32(rc.dims)
    aspect_ratio := dims_f32.x / dims_f32.y
    tan_fov_x := math.tan(scene.cam.fov_x / 2)
    tan_fov_y := tan_fov_x / aspect_ratio

    pixel_to_ray_dir :=
        linalg.matrix4_from_matrix3(scene.cam.basis) *
        linalg.matrix4_scale([3]f32{tan_fov_x, tan_fov_y, 1}) *
        linalg.matrix4_translate([3]f32{-1, -1, 1}) *
        linalg.matrix4_scale(1 / [3]f32{dims_f32.x / 2, dims_f32.y / 2, 1})

    total_tasks: u64
    dim_x := ceil_div(cast(u64)rc.dims.x, RENDERING_TILE_SIZE)
    dim_y := ceil_div(cast(u64)rc.dims.y, RENDERING_TILE_SIZE)
    if rc.samples == max(int) {
        total_tasks = max(u64)
    } else {
        total_tasks = ceil_div(cast(u64)rc.samples, RENDERING_TILE_SAMPLES) *
            dim_x * dim_y
    }

    for {
        id := sync.atomic_add_explicit(tile_id, 1, .Relaxed)
        runtime.random_generator_reset_u64(context.random_generator, id)

        if is_interrupted() do break
        if id >= total_tasks do break

        sample_coord, x_coord, y_coord: u64
        id, y_coord = math.divmod(id, dim_y)
        id, x_coord = math.divmod(id, dim_x)
        sample_coord = id

        num_samples := min(
            RENDERING_TILE_SAMPLES,
            u64(rc.samples) - RENDERING_TILE_SAMPLES * sample_coord
        )
        start_x := RENDERING_TILE_SIZE * cast(u32)x_coord
        end_x := min(
            cast(u32)start_x + RENDERING_TILE_SIZE,
            rc.dims.x
        )
        start_y := RENDERING_TILE_SIZE * cast(u32)y_coord
        end_y := min(
            cast(u32)start_y + RENDERING_TILE_SIZE,
            rc.dims.y
        )

        for sample in 0..<num_samples {
            for px in start_x..<end_x {
                for py in start_y..<end_y {
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
                }
            }
        }
    }
}

render_scene :: proc(rc: Rc, scene: ^Scene, number_of_trials: int = 1) {
    timings := make([]time.Duration, number_of_trials)
    defer delete(timings)

    for trial in 0..<number_of_trials {
        start_instant := time.now()

        num_threads := rc.threads - 1
        tile_id: u64 = 0
        fmt.printfln("Launching %v threads", num_threads)

        threads: sa.Small_Array(256, ^thread.Thread)
        for i in 0..<num_threads {
            t := thread.create_and_start_with_poly_data3(rc, scene, &tile_id, render_task)
            sa.append(&threads, t)
        }

        render_task(rc, scene, &tile_id)

        for t in sa.slice(&threads) {
            thread.join(t)
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

