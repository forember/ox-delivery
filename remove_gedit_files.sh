#!/bin/bash

cd "$(dirname "$0")"
find -name '*~'
read -p 'Remove these files? [y/N]' response
response="$(echo "$response" | tr [:upper:] [:lower:])"
if [ "$response" = 'y' ]; then
    echo 'Deleting files.'
    find -name '*~' -exec rm {} \;
fi
