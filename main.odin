package raytracer

import "core:c"
import "core:sys/posix"
import "core:flags"
import "core:os"
import "core:fmt"
import "core:thread"
import "core:simd"
import "core:sync"
import "core:math/linalg"
import "core:time"

thread_init_barrier: sync.Barrier
async_interrupt: bool = false

is_interrupted :: proc() -> bool {
    return sync.atomic_load_explicit(&async_interrupt, .Relaxed)
}

Rendering_Context :: struct {
    dims: [2]u32,
    pixels: []u32le,
}

Rc :: ^Rendering_Context

create_rendering_context :: proc(dims: [2]u32) -> Rendering_Context {
    pixels := make([]u32le, cast(int)dims.x * cast(int)dims.y)
    return Rendering_Context{
        dims = dims,
        pixels = pixels,
    }
}

destroy_rendering_context :: proc(rendering_context: Rendering_Context) {
    delete(rendering_context.pixels)
}

rc_set_pixel :: proc(rc: Rc, pos: [2]u32, color: [3]f32) {
    assert(pos.x < rc.dims.x)
    assert(pos.y < rc.dims.y)
    i := cast(int)(rc.dims.y - pos.y - 1) * cast(int)rc.dims.x + cast(int)pos.x
    rgb := linalg.to_u32(linalg.round(linalg.clamp(color * 256, 0, 255)))
    data := rgb.r + (rgb.g << 8) + (rgb.b << 16)
    sync.atomic_store_explicit(&rc.pixels[i], cast(u32le)data, .Relaxed)
}

main :: proc() {
    posix.signal(.SIGINT, proc "cdecl" (_: posix.Signal) {
        sync.atomic_store_explicit(&async_interrupt, true, .Relaxed)
    })

    args: struct {
        input_handle: os.Handle `args:"pos=0,required,file=r" usage:"Input scene"`,
        output_file: string `args:"pos=1" usage:"Output image"`,
        debug: bool `usage:"Enable debug window"`,
        times: int `usage:"Number of times to render the scene"`,
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

    number_of_trials := args.times if args.times > 0 else 1
    render_scene(rc, scene, number_of_trials)

    if args.output_file != "" {
        save_result(rc, args.output_file)
    }

    if args.debug {
        thread.join(debug_window_thread)
    }
}

