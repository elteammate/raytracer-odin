package raytracer

import "core:os"
import "core:fmt"
import "core:bufio"
import "core:math/linalg"

read_scene :: proc(file_handle: os.Handle) -> (
    scene: Scene,
    cfg: Rendering_Config,
    error: Maybe(string)
) {
    stream := os.stream_from_handle(file_handle)

    reader: bufio.Reader
    r := &reader
    bufio.reader_init(r, stream)
    defer bufio.reader_destroy(r)

    current_light: ^Light = nil
    current_object: ^Object = nil

    append(&scene.objects, Object{})
    append(&scene.lights, Light{})

    skip_line_whitespace(r)
    for {
        command := read_word_temp(r)
        switch command {
        case "DIMENSIONS":
            cfg.dims.x = cast(u32)read_int(r) or_return
            cfg.dims.y = cast(u32)read_int(r) or_return
        case "RAY_DEPTH":
            cfg.ray_depth = cast(i32)read_int(r) or_return

        case "AMBIENT_LIGHT": scene.ambient = read_3f32(r) or_return
        case "BG_COLOR": scene.objects[0].color = read_3f32(r) or_return

        case "CAMERA_POSITION": scene.cam.pos = read_3f32(r) or_return
        case "CAMERA_RIGHT": scene.cam.basis[0] = read_3f32(r) or_return
        case "CAMERA_UP": scene.cam.basis[1] = read_3f32(r) or_return
        case "CAMERA_FORWARD": scene.cam.basis[2] = read_3f32(r) or_return
        case "CAMERA_FOV_X": scene.cam.fov_x = read_f32(r) or_return

        case "NEW_PRIMITIVE":
            append(&scene.objects, Object{
                rotation = quaternion(x = 0, y = 0, z = 0, w = 1),
                material_kind = .Diffuse,
            })
            current_object = &scene.objects[len(scene.objects) - 1]
        case "PLANE":
            current_object.geometry = Plane{normal = linalg.normalize(read_3f32(r) or_return)}
        case "ELLIPSOID":
            current_object.geometry = Ellipsoid{radii = read_3f32(r) or_return}
        case "BOX":
            current_object.geometry = Box{extent = read_3f32(r) or_return}
        case "POSITION":
            current_object.pos = read_3f32(r) or_return
        case "ROTATION":
            current_object.rotation = read_quat(r) or_return
        case "COLOR":
            current_object.color = read_3f32(r) or_return
        case "IOR":
            current_object.ior = read_f32(r) or_return
        case "METALLIC":
            current_object.material_kind = .Metallic
        case "DIELECTRIC":
            current_object.material_kind = .Dielectric

        case "NEW_LIGHT":
            append(&scene.lights, Light{})
            current_light = &scene.lights[len(scene.lights) - 1]
        case "LIGHT_INTENSITY":
            current_light.intensity = read_3f32(r) or_return
        case "LIGHT_DIRECTION":
            current_light.source.directed.direction = linalg.normalize(read_3f32(r) or_return)
            current_light.kind = .Directed
        case "LIGHT_POSITION":
            current_light.source.point.pos = read_3f32(r) or_return
            current_light.kind = .Point
        case "LIGHT_ATTENUATION":
            current_light.source.point.attenuation = read_3f32(r) or_return
            current_light.kind = .Point
        }
        skip_line_whitespace(r)
        if peek_byte(r) == 0 do break
    }

    return
}

