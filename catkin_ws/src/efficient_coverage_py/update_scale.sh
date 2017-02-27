#!/bin/sh

cd "$(dirname "$0")"

update_scale () {
    name="$1"
    scale="$2"
    python template.py '{"scale":'"$scale"'}' \
        "maps/$name.yaml.template" >"maps/$name.yaml"
    python template.py '{"scale":'"$scale"'}' \
        "world/$name.world.template" >"world/$name.world"
}

update_scale Compare_1 "$1"
update_scale img2 "$1"
