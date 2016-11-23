#!/bin/sh
cd "$(dirname "$0")"
if ! diff -r src/efficient_coverage_py/inputs ../UseLibs/inputs; then
    echo 'src/efficient_coverage_py/inputs is out of sync with'
    read -p '../UseLibs/inputs. Overwrite the former? [y/N]' response
    response="$(echo "$response" | tr [:upper:] [:lower:])"
    if [ "$response" != 'y' ]; then
        exit 1
    fi
fi
echo rm -r src/efficient_coverage_py/inputs
rm -r src/efficient_coverage_py/inputs
cd src/efficient_coverage_py
echo ln -s ../../../UseLibs/inputs inputs
ln -s ../../../UseLibs/inputs inputs
