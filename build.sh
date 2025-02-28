#!/bin/sh
odin build . \
    -o:aggressive -microarch:native \
    -no-bounds-check -no-type-assert -disable-assert \
    -out:raytracer-odin

