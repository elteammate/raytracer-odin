#!/bin/sh
make -C cgltf/src/
/usr/local/odin/odin version &&
rm debug.odin && # well, that's one way to do conditional complication...
/usr/local/odin/odin build . \
    -o:aggressive -microarch:native \
    -no-bounds-check -no-type-assert -disable-assert \
    -define:DEBUG_FEATURES=false \
    -out:raytracer-odin

