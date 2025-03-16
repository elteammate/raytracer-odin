package raytracer

import "core:os"
import "core:c"
import "core:math/linalg"
import "core:strings"
import "core:fmt"
// import stb_image "vendor:stb/image"

Output_Mode :: enum {
    Mean,
    Variance,
    First,
    Last,
    Count,
    Hash,
    NanInf,
}

tone_mapping_aces :: proc(x: $T/[$N]$F) -> T {
    a :: 2.51
    b :: 0.03
    c :: 2.43
    d :: 0.59
    e :: 0.14
    return linalg.clamp((x*(a*x+b))/(x*(c*x+d)+e), 0, 1);
}

get_rgb_image :: proc(rc: Rc, layer: int = 0, mode: Output_Mode = .Mean) -> []byte {
    data := make([]byte, cast(int)rc.dims.x * cast(int)rc.dims.y * 3)
    for y := 0; y < cast(int)rc.dims.y; y += 1 {
        for x := 0; x < cast(int)rc.dims.x; x += 1 {
            i := y * cast(int)rc.dims.x + x
            samples := rc.pixels[layer][i];
            raw: [3]f32
            switch mode {
            case .Mean: raw = samples.total / cast(f32)samples.count
            case .Variance:
                raw = samples.total_squared / cast(f32)samples.count
                raw -= sq(samples.total / cast(f32)samples.count)
            case .First: raw = samples.first
            case .Last: raw = samples.last
            case .Count:
                raw = {
                    cast(f32)samples.count,
                    cast(f32)samples.count / 10,
                    cast(f32)samples.count / 100,
                }
            case .Hash:
                reprs := transmute([3]u32)samples.total
                hash := (reprs * 87334379) & 0xFF
                raw = 1 + linalg.to_f32(hash) / 256
            case .NanInf:
                isnan := linalg.is_nan(samples.total)
                isinf := linalg.is_inf(samples.total)
                raw = tone_mapping_aces(samples.total / cast(f32)samples.count) / 10
                raw.r = isnan.r ? 100 : raw.r
                raw.g = isinf.g ? 100 : raw.g
            }
            tone_mapped := tone_mapping_aces(raw);
            gamma_corrected := linalg.pow(tone_mapped, 1 / 2.2);
            rgb := linalg.to_u8(linalg.round(gamma_corrected * 255));
            // fmt.printfln("%v %v", samples, rgb)
            data[i * 3 + 0] = rgb.r
            data[i * 3 + 1] = rgb.g
            data[i * 3 + 2] = rgb.b
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

