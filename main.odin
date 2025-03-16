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

DEBUG_FEATURES :: #config(DEBUG_FEATURES, true)

thread_init_barrier: sync.Barrier
async_interrupt: bool = false

is_interrupted :: proc() -> bool {
    return sync.atomic_load_explicit(&async_interrupt, .Relaxed)
}

Rendering_Config :: struct {
    dims: [2]u32,
    ray_depth: int,
}

Sample_Stats :: struct {
    first: [3]f32,
    count: u32,
    last: [3]f32,
    total: [3]f32,
    total_squared: [3]f32,
}

NUM_LAYERS :: 10 when DEBUG_FEATURES else 1

Rendering_Context :: struct {
    using cfg: Rendering_Config,
    pixels: [NUM_LAYERS][]Sample_Stats,
}

Rc :: ^Rendering_Context

create_rendering_context :: proc(cfg: Rendering_Config) -> (rc: Rendering_Context) {
    rc.cfg = cfg
    for &layer in rc.pixels {
        layer = make([]Sample_Stats, cast(int)cfg.dims.x * cast(int)cfg.dims.y)
    }
    return
}

destroy_rendering_context :: proc(rendering_context: Rendering_Context) {
    for layer in rendering_context.pixels do delete(layer)
}

rc_set_pixel :: proc(rc: Rc, pos: [2]u32, color: [3]f32, $layer: int) {
    when !DEBUG_FEATURES {
        if layer != 0 do return
    }
    assert(pos.x < rc.dims.x)
    assert(pos.y < rc.dims.y)
    i := cast(int)(rc.dims.y - pos.y - 1) * cast(int)rc.dims.x + cast(int)pos.x
    pixel := &rc.pixels[layer][i]
    if pixel.count == 0 do pixel.first = color
    pixel.count += 1
    pixel.last = color
    pixel.total += color
    pixel.total_squared += color * color
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

    scene, cfg, parse_error := read_scene(args.input_handle)
    if parse_error != nil {
        fmt.panicf("Failed to parse scene: %v", parse_error)
    }
    defer destory_scene(scene)

    rendering_context := create_rendering_context(cfg)
    defer destroy_rendering_context(rendering_context)
    rc := &rendering_context

    sync.barrier_init(&thread_init_barrier, 2)

    when DEBUG_FEATURES {
        debug_window_thread: ^thread.Thread
        if args.debug {
            debug_window_thread = thread.create_and_start_with_poly_data(rc, debug_window_routine)
            sync.barrier_wait(&thread_init_barrier)
        }
    }

    number_of_trials := args.times if args.times > 0 else 1
    render_scene(rc, scene, number_of_trials)

    if args.output_file != "" {
        save_result(rc, args.output_file)
    }

    when DEBUG_FEATURES {
        if args.debug {
            thread.destroy(debug_window_thread)
        }
    }
}

