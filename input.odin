package raytracer

import "core:os"
import "core:fmt"
import "core:bufio"
import "core:slice"
import "core:math/linalg"
import "vendor:cgltf"

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

    // current_light: ^Light = nil
    current_object: ^Object = nil

    append(&scene.objects, Object{})
    // append(&scene.lights, Light{})

    skip_line_whitespace(r)
    for {
        command := read_word_temp(r)
        switch command {
        case "DIMENSIONS":
            cfg.dims.x = cast(u32)read_int(r) or_return
            cfg.dims.y = cast(u32)read_int(r) or_return
        case "RAY_DEPTH":
            cfg.ray_depth = cast(i32)read_int(r) or_return
        case "SAMPLES":
            cfg.samples = read_int(r) or_return

        // case "AMBIENT_LIGHT": scene.ambient = read_3f32(r) or_return
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
        case "TRIANGLE":
            a := read_3f32(r) or_return
            b := read_3f32(r) or_return
            c := read_3f32(r) or_return
            p, u, v := a, b - a, c - a
            n := linalg.normalize(linalg.cross(u, v))
            current_object.geometry = Triangle{p = p, u = u, v = v, n = n}
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
        case "EMISSION":
            current_object.emission = read_3f32(r) or_return

        /*
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
        */
        }
        skip_line_whitespace(r)
        if peek_byte(r) == 0 do break
    }

    return
}

read_gltf :: proc(handle: os.Handle) -> (
    scene: Scene,
    error: Maybe(string)
) {
    data, ok := os.read_entire_file_from_handle(handle)
    if !ok {
        error = "Failed to read .gltf file"
        return
    }
    defer delete(data)
    gltf, parse_result := cgltf.parse({type = .gltf}, raw_data(data), len(data))
    if parse_result != cgltf.result.success {
        error = fmt.tprintf("Failed to parse .gltf file: %v", parse_result)
        return
    }
    defer cgltf.free(gltf)

    append(&scene.objects, Object{})

    populate_scene :: proc(scene: ^Scene, node: ^cgltf.node, parent_transform: matrix[4, 4]f32) -> (error: Maybe(string)) {
        local_transform := linalg.MATRIX4F32_IDENTITY
        cgltf.node_transform_local(node, &local_transform[0, 0])
        transform := parent_transform * local_transform

        if node.camera != nil {
            scene.cam.pos = transform[3].xyz
            scene.cam.basis[0] = transform[0].xyz
            scene.cam.basis[1] = transform[1].xyz
            scene.cam.basis[2] = transform[2].xyz
            scene.cam.fov_x = node.camera.data.perspective.yfov
        }

        if node.mesh != nil {
            for primitive in node.mesh.primitives {
                position_accessor: ^cgltf.accessor
                index_accessor: ^cgltf.accessor = primitive.indices
                for accessor in primitive.attributes {
                    if accessor.name == "POSITION" {
                        position_accessor = accessor.data
                    }
                }
                if position_accessor == nil {
                    return "No position accessor found in mesh primitive"
                }

                num_vertices := index_accessor == nil ? position_accessor.count : index_accessor.count
                for i in 0..<num_vertices / 3 {
                    positions: [3][3]f32
                    for j in 0..<uint(3) {
                        index := index_accessor == nil ? i * 3 + j : cgltf.accessor_read_index(index_accessor, i * 3 + j)
                        if !cgltf.accessor_read_float(position_accessor, i * 3 + j, &positions[j][0], 3) {
                            return "Failed to read position data from accessor"
                        }
                    }
                    positions[0] = (transform * [4]f32{positions[0][0], positions[0][1], positions[0][2], 1}).xyz
                    positions[1] = (transform * [4]f32{positions[1][0], positions[1][1], positions[1][2], 1}).xyz
                    positions[2] = (transform * [4]f32{positions[2][0], positions[2][1], positions[2][2], 1}).xyz
                    append(&scene.objects, Object{
                        geometry = Triangle{
                            p = positions[0],
                            u = positions[1] - positions[0],
                            v = positions[2] - positions[0],
                            n = linalg.normalize(linalg.cross(positions[1] - positions[0], positions[2] - positions[0])),
                        },
                        pos = {0, 0, 0},
                        rotation = quaternion(x = 0, y = 0, z = 0, w = 1),
                        material_kind = .Diffuse,
                    })
                }
            }
        }

        for child in node.children {
            populate_scene(scene, child, transform) or_return
        }
        return
    }

    for node in gltf.scene.nodes {
        populate_scene(&scene, node, linalg.MATRIX4F32_IDENTITY)
    }

    return
}

