ODIN_FLAGS := "-vet-shadowing -microarch:native"

run scene out="out.ppm" width="512" height="512" *rest="":
    odin run . -debug {{ODIN_FLAGS}} \
        -o:aggressive \
        -- {{scene}} {{out}} --debug --continious \
            --width {{width}} --height {{height}} --ray-depth 8 --num-samples 1024 \
            {{rest}}

debug scene out="out.ppm" width="512" height="512" *rest="":
    odin run . -debug {{ODIN_FLAGS}} \
        -o:aggressive \
        -define:EXPENSIVE_DEBUG=true \
        -- {{scene}} {{out}} --debug --continious \
            --width {{width}} --height {{height}} --ray-depth 8 --num-samples 1024 \
            {{rest}}

debug-build:
    odin build . -debug {{ODIN_FLAGS}} -define:EXPENSIVE_DEBUG=true

profile-build:
    odin build . -debug -o:minimal \
        -no-bounds-check -no-type-assert -disable-assert \
        {{ODIN_FLAGS}}

release-build:
    odin build . -debug -o:aggressive \
        -no-bounds-check -no-type-assert -disable-assert \
        {{ODIN_FLAGS}}

sanitize sanitizer scene out="out.ppm":
    odin run . -debug {{ODIN_FLAGS}} \
        -sanitize:{{sanitizer}} \
        -- {{scene}} {{out}}

brrr scene number_of_trials="64":
    odin run . {{ODIN_FLAGS}} \
        -o:aggressive -no-bounds-check -no-type-assert -disable-assert \
        -define:DEBUG_FEATURES=false \
        -- {{scene}} --times {{number_of_trials}}

