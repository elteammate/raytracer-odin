package raytracer

import "core:os"
import "core:net"
import "core:strings"
import "core:fmt"
import "core:bufio"
import "core:slice"
import path "core:path/filepath"
import "core:math/linalg"
import "vendor:cgltf"

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

    append(&scene.trigs, Triangle{})
    append(&scene.materials, Material{})
    texture_cache := make(map[string]Texture)
    defer delete(texture_cache)
    root_path := path.dir(gltf_path)
    defer delete(root_path)

    load_image :: proc(
        root_path: string,
        texture_cache: ^map[string]Texture,
        image: ^cgltf.image,
    ) -> (texture: Texture, error: Maybe(string)) {
        uri, ok := net.percent_decode(string(image.uri), context.temp_allocator)
        if !ok {
            error = fmt.tprintf("Failed to decode image path: %v", image.uri)
            return
        }
        path, err := path.join({root_path, uri}, context.temp_allocator)
        if err != nil {
            error = fmt.tprintf("Failed to resolve image path: %v (%v / %v)", err, root_path, uri)
            return
        }

        if path in texture_cache^ {
            return texture_cache^[path], nil
        }

        texture = load_texture(path) or_return
        texture_cache^[path] = texture
        return texture, nil
    }

    load_sampler :: proc(
        root_path: string,
        texture_cache: ^map[string]Texture,
        texture_view: cgltf.texture_view,
    ) -> (sampler: Sampler, error: Maybe(string)) {
        if texture_view.texture == nil {
            return Sampler{
                texture = nil,
            }, nil
        }

        texture := load_image(root_path, texture_cache, texture_view.texture.image_) or_return
        return Sampler{
            texture = texture,
        }, nil
    }

    populate_scene :: proc(
        scene: ^Scene,
        root_path: string,
        texture_cache: ^map[string]Texture,
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
                normal_accessor: ^cgltf.accessor
                texcoord_accessor: ^cgltf.accessor
                tangent_accessor: ^cgltf.accessor
                index_accessor: ^cgltf.accessor = primitive.indices
                for accessor in primitive.attributes {
                    if accessor.name == "POSITION" {
                        position_accessor = accessor.data
                    }
                    if accessor.name == "NORMAL" {
                        normal_accessor = accessor.data
                    }
                    if accessor.name == "TEXCOORD_0" {
                        texcoord_accessor = accessor.data
                    }
                    if accessor.name == "TANGENT" {
                        tangent_accessor = accessor.data
                    }
                }
                if position_accessor == nil {
                    return "No position accessor found in mesh primitive"
                }

                mat: Material
                metallic_roughness := primitive.material.pbr_metallic_roughness
                color := metallic_roughness.base_color_factor
                mat.color_factor = color.rgb
                mat.color_texture = load_sampler(
                    root_path, texture_cache, metallic_roughness.base_color_texture
                ) or_return
                mat.emission_factor = primitive.material.emissive_factor
                mat.emission_texture = load_sampler(
                    root_path, texture_cache, primitive.material.emissive_texture
                ) or_return
                mat.roughness_factor = metallic_roughness.roughness_factor
                mat.metallic_factor = metallic_roughness.metallic_factor
                mat.metallic_roughness_texture = load_sampler(
                    root_path, texture_cache, metallic_roughness.metallic_roughness_texture
                ) or_return
                mat.normal_texture = load_sampler(
                    root_path, texture_cache, primitive.material.normal_texture
                ) or_return

                if primitive.material.has_emissive_strength {
                    mat.emission_factor *= primitive.material.emissive_strength.emissive_strength
                }

                material_index := len(scene.materials)
                append(&scene.materials, mat)

                num_vertices := index_accessor == nil ? position_accessor.count : index_accessor.count
                for i in 0..<num_vertices / 3 {
                    positions: [3][3]f32
                    normals: [3][3]f32
                    texcoords: [3][2]f32
                    tangents: [3][4]f32
                    for j in 0..<uint(3) {
                        index := index_accessor == nil ? i * 3 + j : cgltf.accessor_read_index(index_accessor, i * 3 + j)
                        if !cgltf.accessor_read_float(position_accessor, index, &positions[j][0], 3) {
                            return "Failed to read position data from accessor"
                        }
                        if normal_accessor != nil {
                            if !cgltf.accessor_read_float(normal_accessor, index, &normals[j][0], 3) {
                                return "Failed to read normal data from accessor"
                            }
                        }
                        if texcoord_accessor != nil {
                            if !cgltf.accessor_read_float(texcoord_accessor, index, &texcoords[j][0], 2) {
                                return "Failed to read UV data from accessor"
                            }
                        }
                        if tangent_accessor != nil {
                            if !cgltf.accessor_read_float(tangent_accessor, index, &tangents[j][0], 4) {
                                return "Failed to read tangent data from accessor"
                            }
                        }
                    }
                    for i in 0..<3 {
                        positions[i] = (transform * [4]f32{positions[i][0], positions[i][1], positions[i][2], 1}).xyz
                        tangents[i].xyz = linalg.normalize(
                            (transform * [4]f32{tangents[i][0], tangents[i][1], tangents[i][2], 0}).xyz
                        )
                    }
                    ng := linalg.normalize(linalg.cross(positions[1] - positions[0], positions[2] - positions[0]))
                    if normal_accessor == nil {
                        for i in 0..<3 {
                            normals[i] = ng
                        }
                    } else {
                        normal_transform := linalg.cofactor(linalg.matrix3_from_matrix4(transform))
                        for i in 0..<3 {
                            normals[i] = linalg.normalize(normal_transform * normals[i])
                        }
                    }

                    append(&scene.trigs, Triangle{
                        p = positions[0],
                        u = positions[1] - positions[0],
                        v = positions[2] - positions[0],
                        n1 = normals[0],
                        n2 = normals[1],
                        n3 = normals[2],
                        tex1 = texcoords[0],
                        tex2 = texcoords[1],
                        tex3 = texcoords[2],
                        tan1 = tangents[0],
                        tan2 = tangents[1],
                        tan3 = tangents[2],
                        ng = ng,
                        material_index = material_index,
                    })
                }
            }
        }

        for child in node.children {
            populate_scene(scene, root_path, texture_cache, child, &transform) or_return
        }
        return
    }

    transform := linalg.MATRIX4F32_IDENTITY
    if gltf.scene != nil {
        for node in gltf.scene.nodes {
            populate_scene(&scene, root_path, &texture_cache, node, &transform) or_return
        }
    } else if gltf.scenes != nil && len(gltf.scenes) > 0 {
        for node in gltf.scenes[0].nodes {
            populate_scene(&scene, root_path, &texture_cache, node, &transform) or_return
        }
    } else {
        for &node in gltf.nodes {
            populate_scene(&scene, root_path, &texture_cache, &node, &transform) or_return
        }
    }

    textures := make([]Texture, len(texture_cache))
    index := 0
    for texture_path, texture in texture_cache {
        textures[index] = texture
        index += 1
    }
    scene.textures = textures

    return
}

