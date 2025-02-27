ODIN_FLAGS := "-vet-shadowing -microarch:native"

run scene:
    odin run . -debug {{ODIN_FLAGS}} \
        -- {{scene}} out.png --debug

sanitize sanitizer scene:
    odin run . -debug {{ODIN_FLAGS}} \
        -sanitize:{{sanitizer}} \
        -- {{scene}} out.png --debug

