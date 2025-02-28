ODIN_FLAGS := "-vet-shadowing -microarch:native"

run scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -- {{scene}} {{out}} --debug

sanitize sanitizer scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -sanitize:{{sanitizer}} \
        -- {{scene}} {{out}}

brrr scene number_of_trials="64":
    odin run . {{ODIN_FLAGS}} \
        -o:aggressive -no-bounds-check -no-type-assert -disable-assert \
        -- {{scene}} --times {{number_of_trials}}

