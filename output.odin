package raytracer

import "core:os"
import "core:c"
import "core:strings"
import "core:fmt"
// import stb_image "vendor:stb/image"

get_rgb_image :: proc(rc: Rc) -> []byte {
    data := make([]byte, cast(int)rc.dims.x * cast(int)rc.dims.y * 3)
    for y := 0; y < cast(int)rc.dims.y; y += 1 {
        for x := 0; x < cast(int)rc.dims.x; x += 1 {
            i := y * cast(int)rc.dims.x + x
            data[i * 3 + 0] = byte(rc.pixels[i])
            data[i * 3 + 1] = byte(rc.pixels[i] >> 8)
            data[i * 3 + 2] = byte(rc.pixels[i] >> 16)
        }
    }
    return data
}

save_result :: proc(rc: Rc, file_path: string) {
    rgb_image := get_rgb_image(rc)
    defer delete(rgb_image)
    c_file_path := strings.clone_to_cstring(file_path)
    defer delete(c_file_path)

    if strings.ends_with(file_path, ".ppm") {
        file_handle, error := os.open(file_path, os.O_CREATE | os.O_WRONLY, 0o664)
        if error != nil {
            fmt.panicf("Failed to open file %v due to %v", file_path, error)
        }
        fmt.fprintf(file_handle, "P6\n%d %d\n255\n", rc.dims.x, rc.dims.y)
        os.write(file_handle, rgb_image)
    // } else if strings.ends_with(file_path, ".png") {
    //     stb_image.write_png(
    //         c_file_path,
    //         cast(c.int)rc.dims.x,
    //         cast(c.int)rc.dims.y,
    //         3,
    //         raw_data(rgb_image),
    //         0,
    //     )
    } else {
        fmt.panicf("Unsupported file format: %v", file_path)
    }
}

