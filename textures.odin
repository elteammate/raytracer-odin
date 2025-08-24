package raytracer

import "core:math/linalg"
import "core:os"
import "core:c"
import "core:fmt"
import stbi "vendor:stb/image"

Texture_Data :: union {
    [^]u8,
    [^]f32,
}

Texture :: struct {
    data: Texture_Data,
    dims: [2]int,
    stride: int,
    channels: u8,
}

Sampler :: struct {
    texture: Maybe(Texture),
}

load_texture :: proc(path: string) -> (texture: Texture, err: Maybe(string)) {
    data, ok := os.read_entire_file_from_filename(path, context.temp_allocator)
    if !ok {
        err = fmt.tprintf("Failed to read texture file: %s", path)
        return
    }

    dims: [2]c.int
    channels: c.int

    image: Texture_Data
    if stbi.is_hdr_from_memory(raw_data(data), cast(c.int)len(data)) != 0 {
        image = stbi.loadf_from_memory(
            raw_data(data), cast(c.int)len(data),
            &dims.x, &dims.y, &channels, 0
        )
        pixels := (transmute([^][3]f32)image.([^]f32))[:dims.x * dims.y]
        min, max, mean: [3]f32
        for p in pixels {
            min = linalg.min(min, p)
            max = linalg.max(max, p)
            mean += p
        }
    } else {
        image = stbi.load_from_memory(
            raw_data(data), cast(c.int)len(data),
            &dims.x, &dims.y, &channels, 0
        )
    }

    if image == nil {
        err = "Failed to parse texture"
        return
    }

    delete(data, context.temp_allocator)

    return Texture{
        data = image,
        dims = {cast(int)dims.x, cast(int)dims.y},
        stride = cast(int)channels * cast(int)dims.x,
        channels = cast(u8)channels,
    }, nil
}

destroy_texture :: proc(texture: Texture) {
    switch data in texture.data {
    case [^]u8:
        stbi.image_free(data)
    case [^]f32:
        stbi.image_free(data)
    }
}

texture_index :: proc(texture: Texture, coords: [2]int, srgb: bool) -> (pixel: [4]f32) {
    assert(coords.x >= 0 && coords.x < int(texture.dims.x) &&
           coords.y >= 0 && coords.y < int(texture.dims.y), "Coordinates out of bounds")

    index := coords.y * texture.stride + coords.x * cast(int)texture.channels

    pixel = {1, 1, 1, 1}
    switch data in texture.data {
    case [^]u8:
        for c in 0..<cast(int)texture.channels {
            pixel[c] = cast(f32)data[index + c] / 255.0
        }
    case [^]f32:
        for c in 0..<cast(int)texture.channels {
            pixel[c] = data[index + c]
        }
    case nil:
        return {1, 1, 1, 1}
    }

    if srgb {
        pixel.rgb = linalg.pow(pixel.rgb, 2.2)
    }

    return pixel
}

texture_sample :: proc(
    sampler: Sampler, coords: [2]f32,
    srgb: bool = false, default: [4]f32 = {1.0, 1.0, 1.0, 1.0},
) -> [4]f32 {
    if sampler.texture == nil {
        return default
    }
    texture := sampler.texture.(Texture)
    pixel_coords := coords * linalg.to_f32(texture.dims)

    coords_low := linalg.floor(pixel_coords)
    coords_high := linalg.ceil(pixel_coords)
    t := pixel_coords - coords_low

    c00 := linalg.to_int(coords_low) %% texture.dims
    c11 := linalg.to_int(coords_high) %% texture.dims
    c01 := [2]int{c00.x, c11.y}
    c10 := [2]int{c11.x, c00.y}

    p00 := texture_index(texture, c00, srgb)
    p01 := texture_index(texture, c01, srgb)
    p10 := texture_index(texture, c10, srgb)
    p11 := texture_index(texture, c11, srgb)

    return linalg.lerp(
        linalg.lerp(p00, p01, t.y),
        linalg.lerp(p10, p11, t.y),
        t.x,
    )
}

