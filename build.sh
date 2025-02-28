#!/bin/sh
make -C "/usr/local/odin/vendor/stb/src"
/usr/local/odin/odin build . \
    -o:aggressive -microarch:native \
    -no-bounds-check -no-type-assert -disable-assert \
    -out:raytracer-odin

