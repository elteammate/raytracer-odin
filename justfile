ODIN_FLAGS := "-vet-shadowing -microarch:native"

run scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -o:aggressive \
        -- {{scene}} {{out}} --debug

debug scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -o:aggressive \
        -define:EXPENSIVE_DEBUG=true \
        -- {{scene}} {{out}} --debug

sanitize sanitizer scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -sanitize:{{sanitizer}} \
        -- {{scene}} {{out}}

brrr scene number_of_trials="64":
    odin run . {{ODIN_FLAGS}} \
        -o:aggressive -no-bounds-check -no-type-assert -disable-assert \
        -define:DEBUG_FEATURES=false \
        -- {{scene}} --times {{number_of_trials}}

