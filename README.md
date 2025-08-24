# CPU Raytracer

A renderer I have been developing during the raytracing course in SPbU.

This repo is a self-contained (unidirectional) CPU-only raytracer
written in [Odin programming language](https://odin-lang.org/).

The design and implementation are based on [@lisyarus raytracing course](https://github.com/lisyarus/raytracing-course-slides).

![a funny image containing a 3D scene full of memes rendered with ray tracing](demo-image.png)

## Building & Running

This project uses [just](https://github.com/casey/just), a handy command runner.

To build in release mode, simply run
```sh
just release-build
```
You can check the [justfile] for more commands.

The resulting binary will be named by the repository directory name,
`raytracer-odin` by default.
Provide a glTF file to render and additional rendering parameters from CLI:
```sh
./raytracer-odin <path-to-gltf> <output-file-name> \
    --width <result image width> \
    --height <result image height> \
    --ray-depth <max raytracing recursion depth> \
    --num-samples <number-of-samples-per-pixel> \
    [--env-map <path-to-env-map>] \
    [--debug] [--continious]
```
If you add `--debug` flag, it will additionally open a window you can preview
intermediate output in.

`--continious` flag makes raytracer ignore number of samples provided and
and it will instead render continuously until it is interrupted with `CTRL+C`
or the debug window is closed.

If compiled with `EXPENSIVE_DEBUG=true` feature, the debug window will get
additional functionality. By hovering over the pixel, you can see some path
which were started from that pixel. This can be disabled by pressing `Z`.
You will also be able to preview the internal acceleration data structure.
To configure which layer is visible, use `X` and `C` buttons.

There is also a concept of debugging layers. By default,
the image is rendered on layer 0, which you can switch to by pressing
`1` on the keyboard. However, when `rc_set_*` function is called in the
implementation, the additional sample is recorded on a given layer.
This layer can be then viewed by pressing the corresponding number key.

Finally, each layer not only collects information about the average of 
the samples, but also tracks additional data. By pressing `Q-W-E-R-T-...`
you can switch between different preview features, and, for example,
render the value of the last sample, the variance of the sample, or
highlight NaNs and infinities, and so on.

## Input file considerations

Note that a very limited subset of glTF features are supported.

If you use blender, make sure to configure the export correctly.
All images must be exported as JPEG or PNG.
Make sure to apply all modifiers and limit the scope of your materials
to the subset described in [glTF spec](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html)
under [BRDF implementation section](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation).
Bake textures if needed.

