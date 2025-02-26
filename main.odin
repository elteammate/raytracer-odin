package raytracer

import "core:c"
import "core:flags"
import "core:os"
import "core:fmt"
import "core:thread"
import "core:simd"
import "core:sync"
import "core:math/linalg"
import "core:time"

import "vendor:sdl2"
import sdl2_image "vendor:sdl2/image"

thread_init_barrier: sync.Barrier

Rendering_Context :: struct {
    dims: [2]u16,
    pixels: []u32le,
}

Rc :: ^Rendering_Context

async_interrupt: bool = false

create_rendering_context :: proc(dims: [2]u16) -> Rendering_Context {
    pixels := make([]u32le, cast(int)dims.x * cast(int)dims.y)
    return Rendering_Context{
        dims = dims,
        pixels = pixels,
    }
}

destroy_rendering_context :: proc(rendering_context: Rendering_Context) {
    delete(rendering_context.pixels)
}

rc_set_pixel :: proc(rc: Rc, pos: [2]u16, color: [3]f32) {
    assert(pos.x < rc.dims.x)
    assert(pos.y < rc.dims.y)
    i := cast(int)pos.y * cast(int)rc.dims.x + cast(int)pos.x
    rgb := linalg.to_u32(linalg.clamp(color * 256, 0, 255))
    data := rgb.r + (rgb.g << 8) + (rgb.b << 16)
    sync.atomic_store_explicit(&rc.pixels[i], cast(u32le)data, .Relaxed)
}

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

Scene :: struct {
    bg_color: [3]f32,
    cam_pos: [3]f32,
    cam_right, cam_up, cam_forward: [3]f32,
    cam_fov_x: f32,
    objects: [dynamic]Object,
}

destory_scene :: proc(s: Scene) {
    delete(s.objects)
}

Ray :: struct {
    o, d: [3]f32,
}

main :: proc() {
    args: struct {
        input_handle: os.Handle `args:"pos=0,required,file=r" usage:"Input scene"`,
        output_file: string `args:"pos=1" usage:"Output image"`,
        debug: bool `usage:"Enable debug window"`,
    }
    flags.parse_or_exit(&args, os.args, .Unix)
    defer os.close(args.input_handle)

    scene, dims, parse_error := read_scene(args.input_handle)
    if parse_error != nil {
        fmt.panicf("Failed to parse scene: %v", parse_error)
    }
    defer destory_scene(scene)

    rendering_context := create_rendering_context(dims)
    defer destroy_rendering_context(rendering_context)
    rc := &rendering_context

    sync.barrier_init(&thread_init_barrier, 2)

    debug_window_thread: ^thread.Thread
    if args.debug {
        debug_window_thread = thread.create_and_start_with_poly_data(rc, debug_window_routine)
        sync.barrier_wait(&thread_init_barrier)
    }

    for x in 0..<rendering_context.dims.x {
        for y in 0..<rendering_context.dims.y {
            rc_set_pixel(rc, {x, y}, {1.0, 1.0, 0.5})
            // time.sleep(time.Nanosecond)
        }
    }

    if args.output_file != "" {
        save_result(rc, args.output_file)
    }

    if args.debug {
        thread.join(debug_window_thread)
    }
}

