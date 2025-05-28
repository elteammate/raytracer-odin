package raytracer

import "core:os"
import "core:strings"
import "core:fmt"
import "core:bufio"
import "core:slice"
import "core:math/linalg"
import "cgltf"

read_gltf :: proc(gltf_path: string) -> (
    scene: Scene,
    error: Maybe(string)
) {
    handle, file_open_error := os.open(gltf_path)
    if file_open_error != nil {
        error = fmt.tprintf("Failed to open input file: %v", file_open_error)
        return
    }
    data, read_error := os.read_entire_file_from_handle_or_err(handle)
    if read_error != nil {
        error = fmt.tprintf("%v", read_error)
        return
    }
    defer delete(data)
    gltf, parse_result := cgltf.parse({type = .gltf}, raw_data(data), len(data))
    if parse_result != cgltf.result.success {
        error = fmt.tprintf("Failed to parse .gltf file: %v", parse_result)
        return
    }
    defer cgltf.free(gltf)

    buffer_load_result := cgltf.load_buffers(
        {}, gltf, strings.clone_to_cstring(gltf_path, context.temp_allocator)
    )
    if buffer_load_result != cgltf.result.success {
        error = fmt.tprintf("Failed to load buffers from .gltf file: %v", buffer_load_result)
        return
    }

    append(&scene.objects, Object{})

    populate_scene :: proc(
        scene: ^Scene,
        node: ^cgltf.node,
        parent_transform: ^matrix[4, 4]f32
    ) -> (error: Maybe(string)) {
        local_transform: matrix[4, 4]f32 = linalg.MATRIX4F32_IDENTITY
        cgltf.node_transform_local(node, &local_transform[0, 0])
        transform := parent_transform^ * local_transform

        if node.camera != nil {
            scene.cam.pos = transform[3].xyz
            scene.cam.basis[0] = transform[0].xyz
            scene.cam.basis[1] = transform[1].xyz
            scene.cam.basis[2] = -transform[2].xyz
            scene.cam.fov_x = node.camera.data.perspective.yfov
        }

        if node.mesh != nil {
            for &primitive in node.mesh.primitives {
                primitive: ^cgltf.primitive = &primitive
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

                material_kind := Material_Kind.Diffuse
                color: [4]f32 = primitive.material.pbr_metallic_roughness.base_color_factor
                ior: f32
                emission: [3]f32 = primitive.material.emissive_factor
                if color.a < 1.0 {
                    material_kind = .Dielectric
                    ior = 1.5
                }
                if primitive.material.pbr_metallic_roughness.metallic_factor > 0.0 {
                    material_kind = .Metallic
                }
                if primitive.material.has_emissive_strength {
                    emission *= primitive.material.emissive_strength.emissive_strength
                }

                num_vertices := index_accessor == nil ? position_accessor.count : index_accessor.count
                for i in 0..<num_vertices / 3 {
                    positions: [3][3]f32
                    for j in 0..<uint(3) {
                        index := index_accessor == nil ? i * 3 + j : cgltf.accessor_read_index(index_accessor, i * 3 + j)
                        if !cgltf.accessor_read_float(position_accessor, index, &positions[j][0], 3) {
                            return "Failed to read position data from accessor"
                        }
                    }
                    positions[0] = (transform * [4]f32{positions[0][0], positions[0][1], positions[0][2], 1}).xyz
                    positions[1] = (transform * [4]f32{positions[1][0], positions[1][1], positions[1][2], 1}).xyz
                    positions[2] = (transform * [4]f32{positions[2][0], positions[2][1], positions[2][2], 1}).xyz
                    append(&scene.objects, Object{
                        trig = Triangle{
                            p = positions[0],
                            u = positions[1] - positions[0],
                            v = positions[2] - positions[0],
                            n = linalg.normalize(linalg.cross(positions[1] - positions[0], positions[2] - positions[0])),
                        },
                        material_kind = material_kind,
                        color = color.rgb,
                        ior = ior,
                        emission = emission,
                    })
                }
            }
        }

        for child in node.children {
            populate_scene(scene, child, &transform) or_return
        }
        return
    }

    gltf_scene := gltf.scene == nil ? &gltf.scenes[0] : gltf.scene
    for node in gltf_scene.nodes {
        transform := linalg.MATRIX4F32_IDENTITY
        populate_scene(&scene, node, &transform)
    }

    return
}

