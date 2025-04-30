package raytracer

import "core:c"
import "core:sys/posix"
import "core:flags"
import "core:os"
import "core:fmt"
import "core:thread"
import "core:simd"
import "core:sync"
import sa "core:container/small_array"
import "core:math/linalg"
import "core:time"

DEBUG_FEATURES :: #config(DEBUG_FEATURES, true)
EXPENSIVE_DEBUG :: #config(EXPENSIVE_DEBUG, false)

thread_init_barrier: sync.Barrier
async_interrupt: bool = false

is_interrupted :: proc() -> bool {
    return sync.atomic_load_explicit(&async_interrupt, .Relaxed)
}

Rendering_Config :: struct {
    dims: [2]u32,
    ray_depth: i32,
    samples: int,
    threads: int,
}

Sample_Stats :: struct {
    first: [3]f32,
    count: u32,
    last: [3]f32,
    total: [3]f32,
    total_squared: [3]f32,
}

Cast_Info :: struct {
    ray: Ray,
    t: f32,
    color: [3]f32,
}

NUM_LAYERS :: 10 when DEBUG_FEATURES else 1

Debug_Line :: struct {
    a, b, color: [3]f32,
    tag: u32,
}

Rendering_Context :: struct #no_copy {
    using cfg: Rendering_Config,
    pixels: [NUM_LAYERS][]Sample_Stats,
    ray_logs: []sa.Small_Array(256, Cast_Info),
    debug_lines: [dynamic]Debug_Line,
    lock: sync.Mutex,
}

Rc :: ^Rendering_Context

create_rendering_context :: proc(cfg: Rendering_Config) -> (rc: Rendering_Context) {
    rc.cfg = cfg
    for &layer in rc.pixels {
        layer = make([]Sample_Stats, cast(int)cfg.dims.x * cast(int)cfg.dims.y)
    }
    when EXPENSIVE_DEBUG {
        rc.ray_logs = make(type_of(rc.ray_logs), cast(int)cfg.dims.x * cast(int)cfg.dims.y)
    }
    when DEBUG_FEATURES {
        rc.debug_lines = make(type_of(rc.debug_lines), 0, 1024)
    }
    return rc
}

destroy_rendering_context :: proc(rc: Rc) {
    for layer in rc.pixels do delete(layer)
}

rc_set_pixel :: proc(rc: Rc, pos: [2]u32, color: [3]f32, layer: int) {
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

debug_rc_set_float :: proc(color: [3]f32, layer: int) {
    when DEBUG_FEATURES {
        rc_set_pixel(debug_info.rc, debug_info.pixel, color, layer)
    }
}

debug_rc_set_bool :: proc(value: bool, layer: int) {
    when DEBUG_FEATURES {
        rc_set_pixel(debug_info.rc, debug_info.pixel, f32(u8(value)), layer)
    }
}

debug_rc_set :: proc{debug_rc_set_float, debug_rc_set_bool}

debug_log_ray :: proc(info: Cast_Info) {
    when EXPENSIVE_DEBUG {
        using debug_info
        i := cast(int)(rc.dims.y - pixel.y - 1) * cast(int)rc.dims.x + cast(int)pixel.x
        sa.append(&debug_info.rc.ray_logs[i], info)
    }
}

rc_log_line :: proc(rc: Rc, a, b: [3]f32, color: [3]f32 = {1, 1, 1}, tag: u32 = 0) {
    when DEBUG_FEATURES {
        sync.lock(&rc.lock)
        defer sync.unlock(&rc.lock)
        append(&rc.debug_lines, Debug_Line{a, b, color, tag})
    }
}

rc_log_aabb :: proc(rc: Rc, aabb: AABB, color: [3]f32 = {1, 1, 1}, tag: u32 = 0) {
    when DEBUG_FEATURES {
        sync.lock(&rc.lock)
        defer sync.unlock(&rc.lock)
        a := aabb.lo
        b := aabb.hi
        append(
            &rc.debug_lines,
            Debug_Line{{a.x, a.y, a.z}, {b.x, a.y, a.z}, color, tag},
            Debug_Line{{a.x, b.y, a.z}, {b.x, b.y, a.z}, color, tag},
            Debug_Line{{a.x, a.y, b.z}, {b.x, a.y, b.z}, color, tag},
            Debug_Line{{a.x, b.y, b.z}, {b.x, b.y, b.z}, color, tag},
            Debug_Line{{a.x, a.y, a.z}, {a.x, b.y, a.z}, color, tag},
            Debug_Line{{b.x, a.y, a.z}, {b.x, b.y, a.z}, color, tag},
            Debug_Line{{a.x, a.y, b.z}, {a.x, b.y, b.z}, color, tag},
            Debug_Line{{b.x, a.y, b.z}, {b.x, b.y, b.z}, color, tag},
            Debug_Line{{a.x, a.y, a.z}, {a.x, a.y, b.z}, color, tag},
            Debug_Line{{b.x, a.y, a.z}, {b.x, a.y, b.z}, color, tag},
            Debug_Line{{a.x, b.y, a.z}, {a.x, b.y, b.z}, color, tag},
            Debug_Line{{b.x, b.y, a.z}, {b.x, b.y, b.z}, color, tag},
        )
    }
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
        continious: bool `usage:"Ignore sample limit and render until interrupted"`,
        threads: int `usage:"Number of threads to use"`,
    }

    flags.parse_or_exit(&args, os.args, .Unix)
    defer os.close(args.input_handle)

    scene, cfg, parse_error := read_scene(args.input_handle)
    if parse_error != nil {
        fmt.panicf("Failed to parse scene: %v", parse_error)
    }
    defer destory_scene(&scene)

    if args.continious do cfg.samples = max(int)
    if args.threads > 0 {
        cfg.threads = args.threads
    } else {
        cfg.threads = max(os.processor_core_count(), 1)
    }

    rendering_context := create_rendering_context(cfg)
    rc := &rendering_context
    defer destroy_rendering_context(rc)

    sync.barrier_init(&thread_init_barrier, 2)

    when DEBUG_FEATURES {
        debug_window_thread: ^thread.Thread
        if args.debug {
            debug_window_thread = thread.create_and_start_with_poly_data2(
                rc, &scene,
                debug_window_routine
            )
            sync.barrier_wait(&thread_init_barrier)
        }
    }

    finish_scene(rc, &scene)

    number_of_trials := args.times if args.times > 0 else 1
    render_scene(rc, &scene, number_of_trials)

    if args.output_file != "" {
        save_result(rc, args.output_file)
    }

    when DEBUG_FEATURES {
        if args.debug {
            thread.destroy(debug_window_thread)
        }
    }
}

