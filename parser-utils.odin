package raytracer

import "core:bufio"
import "core:fmt"
import "core:strconv"
import "core:unicode"

LOCAL_BUFFER_CAPACITY :: 512
@thread_local LOCAL_BUFFER: [LOCAL_BUFFER_CAPACITY]byte

peek_byte :: proc(r: ^bufio.Reader) -> byte {
    data, err := bufio.reader_peek(r, 1)
    if err != nil do return 0
    return data[0]
}

read_byte :: proc(r: ^bufio.Reader) -> byte {
    data, err := bufio.reader_read_byte(r)
    if err != nil do return 0
    return data
}

is_whitespace :: proc(c: byte) -> bool {
    return c == ' ' || c == '\t' || c == '\r'
}

is_end_of_line :: proc(c: byte) -> bool {
    return c == '\n' || c == 0
}

skip_whitespace :: proc(r: ^bufio.Reader) {
    for {
        b := peek_byte(r)
        if b == 0 || !is_whitespace(b) do break
        read_byte(r)
    }
}

read_word_temp :: proc(r: ^bufio.Reader) -> string {
    skip_whitespace(r)

    result := LOCAL_BUFFER[:]
    i := 0

    for ; i < LOCAL_BUFFER_CAPACITY; i += 1 {
        b := peek_byte(r)
        if is_whitespace(b) || is_end_of_line(b) do break
        result[i] = b
        read_byte(r)
    }
    return string(result[:i])
}

read_line_temp :: proc(r: ^bufio.Reader) -> string {
    result := LOCAL_BUFFER[:]
    i := 0

    for ; i < LOCAL_BUFFER_CAPACITY; i += 1 {
        b := peek_byte(r)
        if is_end_of_line(b) do break
        result[i] = b
        read_byte(r)
    }
    if i != LOCAL_BUFFER_CAPACITY do read_byte(r)
    return string(result[:i])
}

skip_line :: proc(r: ^bufio.Reader) {
    for {
        b := read_byte(r)
        if is_end_of_line(b) do break
    }
}

skip_line_whitespace :: proc(r: ^bufio.Reader) {
    for {
        b := peek_byte(r)
        if b != 0 && (is_end_of_line(b) || is_whitespace(b)) do read_byte(r)
        else do break
    }
}

expect_byte :: proc(r: ^bufio.Reader, expected: byte) -> Maybe(string) {
    b := read_byte(r)
    if b == expected do return nil
    return fmt.tprintf("Expected '%c' but got '%c'", expected, b)
}

read_f32 :: proc(r: ^bufio.Reader) -> (f32, Maybe(string)) {
    skip_whitespace(r)

    buf := LOCAL_BUFFER[:]
    i := 0

    for ; i < LOCAL_BUFFER_CAPACITY; i += 1 {
        b := peek_byte(r)
        if b == '.' || unicode.is_digit(rune(b)) || b == '-' || b == '+' || b == 'e' {
            buf[i] = b
            read_byte(r)
        } else {
            break
        }
    }

    if i == 0 do return 0, fmt.tprintf("Expected a floating-point number but got '%c'", peek_byte(r))
    value, ok := strconv.parse_f32(string(buf[:i]))
    if !ok do return 0, fmt.tprintf("Failed to parse floating-point number: %v", string(buf[:i]))
    return value, nil
}

read_int :: proc(r: ^bufio.Reader) -> (int, Maybe(string)) {
    skip_whitespace(r)

    buf := LOCAL_BUFFER[:]
    seen_sign := false
    seen_digits := false
    i := 0

    for ; i < LOCAL_BUFFER_CAPACITY; i += 1 {
        b := peek_byte(r)
        if b == '-' || b == '+' {
            if seen_sign || seen_digits do break
            seen_sign = true
        } else if unicode.is_digit(rune(b)) {
            seen_digits = true
        } else {
            break
        }
        buf[i] = b
        read_byte(r)
    }

    if !seen_digits do return 0, fmt.tprintf("Expected an integer but got '%c'", peek_byte(r))

    value, ok := strconv.parse_int(string(buf[:i]), 10)
    if !ok do return 0, fmt.tprintf("Failed to parse integer: %v", string(buf[:i]))
    return value, nil
}

read_3f32 :: proc(r: ^bufio.Reader) -> (vec: [3]f32, error: Maybe(string)) {
    vec.x = read_f32(r) or_return
    vec.y = read_f32(r) or_return
    vec.z = read_f32(r) or_return
    return
}

read_quat :: proc(r: ^bufio.Reader) -> (quat: quaternion128, error: Maybe(string)) {
    quat.x = read_f32(r) or_return
    quat.y = read_f32(r) or_return
    quat.z = read_f32(r) or_return
    quat.w = read_f32(r) or_return
    return
}
