package raytracer

import "core:c"
import "core:fmt"
import sa "core:container/small_array"
import "core:sync"
import "core:math/linalg"
import "core:math"

import "vendor:sdl2"

debug_window_routine :: proc(rc: Rc, scene: ^Scene) {
    sync.barrier_wait(&thread_init_barrier)

    if error := sdl2.Init({.VIDEO}); error < 0 {
        fmt.eprintfln("Failed to initialize SDL (error code %d)", error)
        return
    }
    defer sdl2.Quit()

    window := sdl2.CreateWindow(
        "Raytracer",
        100, 100,
        cast(c.int)rc.dims.x, cast(c.int)rc.dims.y,
        {},
    )
    if window == nil {
        fmt.eprintfln("Failed to create SDL window")
        return
    }
    defer sdl2.DestroyWindow(window)

    renderer := sdl2.CreateRenderer(window, -1, {})
    if renderer == nil {
        fmt.eprintfln("Failed to create SDL renderer")
        return
    }
    defer sdl2.DestroyRenderer(renderer)

    layer: int = 0
    mode: Output_Mode = .Mean

    window_loop: for !is_interrupted() {
        for event: sdl2.Event = ---; sdl2.PollEvent(&event); {
            #partial switch event.type {
            case .QUIT:
                break window_loop
            case .KEYDOWN:
                #partial switch event.key.keysym.sym {
                case .ESCAPE:
                    break window_loop
                case .NUM1: layer = 0
                case .NUM2: layer = 1
                case .NUM3: layer = 2
                case .NUM4: layer = 3
                case .NUM5: layer = 4
                case .NUM6: layer = 5
                case .NUM7: layer = 6
                case .NUM8: layer = 7
                case .NUM9: layer = 8
                case .NUM0: layer = 9
                case .Q: mode = .Mean
                case .W: mode = .Variance
                case .E: mode = .First
                case .R: mode = .Last
                case .T: mode = .Weight
                case .Y: mode = .Count
                case .U: mode = .Hash
                case .I: mode = .NanInf
                }
                fmt.printfln("Layer: %d, Mode: %s", layer, mode)
            }
        }

        output_buffer := get_rgb_image(rc, layer, mode)
        defer delete(output_buffer)

        depth :: 3
        surface := sdl2.CreateRGBSurfaceWithFormatFrom(
            raw_data(output_buffer),
            width = cast(c.int)rc.dims.x, height = cast(c.int)rc.dims.y,
            depth = depth, pitch = depth * cast(c.int)rc.dims.x,
            format = cast(u32)sdl2.PixelFormatEnum.RGB24,
        )
        defer sdl2.FreeSurface(surface)

        texture := sdl2.CreateTextureFromSurface(renderer, surface)
        defer sdl2.DestroyTexture(texture)

        sdl2.RenderClear(renderer)
        rect := sdl2.Rect{
            x = 0, y = 0,
            w = cast(c.int)rc.dims.x, h = cast(c.int)rc.dims.y,
        }
        sdl2.RenderCopy(renderer, texture, nil, &rect)

        when EXPENSIVE_DEBUG {
            mouse_position: [2]c.int
            sdl2.GetMouseState(&mouse_position.x, &mouse_position.y)
            i := mouse_position.y * cast(c.int)rc.dims.x + mouse_position.x
            #reverse for cast_ in sa.slice(&rc.ray_logs[i]) {
                p1, p2 := line_to_screen(
                    rc, &scene.cam,
                    cast_.ray.o,
                    cast_.ray.o + cast_.ray.d * min(cast_.t, 1e2)
                )
                if !linalg.is_nan(p1.x) {
                    if cast_.level == -1 {
                        sdl2.SetRenderDrawColor(renderer, 255, 255, 0, 120)
                    } else {
                        sdl2.SetRenderDrawColor(
                            renderer,
                            cast(u8)(cast_.level & 1) * 255,
                            cast(u8)((cast_.level >> 1) & 1) * 255,
                            cast(u8)((cast_.level >> 2) & 1) * 255,
                            255
                        )
                    }
                    sdl2.RenderDrawLine(
                        renderer,
                        cast(c.int)p1.x, cast(c.int)p1.y,
                        cast(c.int)p2.x, cast(c.int)p2.y,
                    )
                }
            }
        }

        sdl2.RenderPresent(renderer)
    }

    sync.atomic_store_explicit(&async_interrupt, true, .Relaxed)
}

