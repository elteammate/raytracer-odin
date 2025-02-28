ODIN_FLAGS := "-vet-shadowing -microarch:native"

run scene:
    odin run . -debug {{ODIN_FLAGS}} \
        -- {{scene}} out.png --debug

sanitize sanitizer scene:
    odin run . -debug {{ODIN_FLAGS}} \
        -sanitize:{{sanitizer}} \
        -- {{scene}} out.png --debug

brrr scene number_of_trials="64":
    odin run . {{ODIN_FLAGS}} \
        -o:aggressive -no-bounds-check -no-type-assert -disable-assert \
        -- {{scene}} --times {{number_of_trials}}

