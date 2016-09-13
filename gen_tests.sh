#!/bin/bash

caller_cwd="$(readlink -e .)"
script_dir="$(readlink -e "$(dirname "$0")")"
input_dir="$script_dir/UseLibs/inputs"
output_dir="$script_dir/gen_tests_out"
test_k_list=(1 2 4 5 6 7 8 16 32)

gen_tests_for_image () {
    echo "Generating tests for $4"
    d="$output_dir/${4}.d"
    mkdir -p "$d"
    cd "$d"
    for k in "${test_k_list[@]}"; do
        mkdir -p "$k"
        cd "$k"
        echo "IN $(readlink -e .)" > oxdel.log
        echo "  CALL: " afrl-oxdel "$1" "$4" "$k" 0 >> oxdel.log
        echo afrl-oxdel "$1" "$4" "$k" 0
        afrl-oxdel "$1" "$4" "$k" 0 &>> oxdel.log
        cd "$d"
    done
    cd "$caller_cwd"
}

gtfi_default () {
    gen_tests_for_image "$input_dir" "$output_dir" "$test_k_list" "$@"
}

gtfi_default bad_world.png
gtfi_default cave.png
gtfi_default Compare_1.png
gtfi_default greg_cottage_map.png
gtfi_default img0.png
gtfi_default img1.png
gtfi_default img2.png
gtfi_default img3.png
gtfi_default img4.png
gtfi_default img5.png
gtfi_default world1.png
