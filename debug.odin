package raytracer

import "core:c"
import "core:fmt"
import "core:sync"

import "vendor:sdl2"

debug_window_routine :: proc(rc: Rc) {
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
                case .T: mode = .Count
                case .Y: mode = .Hash
                case .U: mode = .NanInf
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
        sdl2.RenderPresent(renderer)
    }

    sync.atomic_store_explicit(&async_interrupt, true, .Relaxed)
}

