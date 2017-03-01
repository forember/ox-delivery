#!/bin/sh

cd "$(dirname "$0")"

for i in `seq 1 8`; do
    python template.py '{"k":'"$i"'}' \
        'launch/efficient_coverage.launch.template' \
            >"launch/efficient_coverage_$i.launch"
done
