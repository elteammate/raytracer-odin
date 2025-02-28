package raytracer

import "core:os"
import "core:fmt"
import "core:bufio"

read_scene :: proc(file_handle: os.Handle) -> (
    scene: Scene,
    dims: [2]u32,
    error: Maybe(string)
) {
    stream := os.stream_from_handle(file_handle)

    reader: bufio.Reader
    r := &reader
    bufio.reader_init(r, stream)
    defer bufio.reader_destroy(r)

    current: ^Object = nil

    append(&scene.objects, Object{})

    skip_line_whitespace(r)
    for {
        command := read_word_temp(r)
        switch command {
        case "DIMENSIONS":
            dims.x = cast(u32)read_int(r) or_return
            dims.y = cast(u32)read_int(r) or_return
        case "BG_COLOR": scene.objects[0].color = read_3f32(r) or_return
        case "CAMERA_POSITION": scene.cam.pos = read_3f32(r) or_return
        case "CAMERA_RIGHT": scene.cam.basis[0] = read_3f32(r) or_return
        case "CAMERA_UP": scene.cam.basis[1] = read_3f32(r) or_return
        case "CAMERA_FORWARD": scene.cam.basis[2] = read_3f32(r) or_return
        case "CAMERA_FOV_X": scene.cam.fov_x = read_f32(r) or_return
        case "NEW_PRIMITIVE":
            append(&scene.objects, Object{
                conj_rotation = quaternion(x = 0, y = 0, z = 0, w = 1),
            })
            current = &scene.objects[len(scene.objects) - 1]
        case "PLANE": current.geometry = Plane{normal = read_3f32(r) or_return}
        case "ELLIPSOID": current.geometry = Ellipsoid{radii = read_3f32(r) or_return}
        case "BOX": current.geometry = Box{extent = read_3f32(r) or_return}
        case "POSITION": current.pos = read_3f32(r) or_return
        case "ROTATION": current.conj_rotation = conj(read_quat(r) or_return)
        case "COLOR": current.color = read_3f32(r) or_return
        }
        skip_line_whitespace(r)
        if peek_byte(r) == 0 do break
    }

    return
}

