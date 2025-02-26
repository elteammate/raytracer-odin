package raytracer

import "core:os"
import "core:fmt"
import "core:bufio"

read_scene :: proc(file_handle: os.Handle) -> (
    scene: Scene,
    dims: [2]u16,
    error: Maybe(string)
) {
    stream := os.stream_from_handle(file_handle)

    reader: bufio.Reader
    r := &reader
    bufio.reader_init(r, stream)
    defer bufio.reader_destroy(r)

    current: ^Object = nil

    skip_line_whitespace(r)
    for {
        command := read_word_temp(r)
        switch command {
        case "DIMENSIONS":
            dims.x = cast(u16)read_int(r) or_return
            dims.y = cast(u16)read_int(r) or_return
        case "BG_COLOR": scene.bg_color = read_3f32(r) or_return
        case "CAMERA_POSITION": scene.cam_pos = read_3f32(r) or_return
        case "CAMERA_UP": scene.cam_up = read_3f32(r) or_return
        case "CAMERA_RIGHT": scene.cam_right = read_3f32(r) or_return
        case "CAMERA_FORWARD": scene.cam_forward = read_3f32(r) or_return
        case "CAMERA_FOV_X": scene.cam_fov_x = read_f32(r) or_return
        case "NEW_PRIMITIVE":
            append(&scene.objects, Object{
                rotation = quaternion(x = 0, y = 0, z = 0, w = 1),
            })
            current = &scene.objects[len(scene.objects) - 1]
        case "PLANE": current.geometry = Plane{normal = read_3f32(r) or_return}
        case "ELLIPSOID": current.geometry = Ellipsoid{radii = read_3f32(r) or_return}
        case "BOX": current.geometry = Box{extent = read_3f32(r) or_return}
        case "POSITION": current.pos = read_3f32(r) or_return
        case "ROTATION": current.rotation = read_quat(r) or_return
        case "COLOR": current.color = read_3f32(r) or_return
        }
        skip_line_whitespace(r)
        if peek_byte(r) == 0 do break
    }

    return
}

