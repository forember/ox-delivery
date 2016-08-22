#!/bin/bash

print_usage () {
    echo 'usage: force_bash.sh --yes'
    echo
    echo '      Convert scripts to use /bin/bash'
    echo
    echo 'usage: force_bash.sh --no'
    echo
    echo '      Convert scripts to use /bin/sh'
    exit 1
}

if [ -z "$1" ]; then
    print_usage
fi

cd "$(dirname "$0")"

if [ "$1" = '--yes' ]; then
    bash clean_all.sh
    find -name '*.sh' \
        -exec sed -i -e 's|#!/bin/bash|#!/bin/bash|' {} +
elif [ "$1" = '--no' ]; then
    sh clean_all.sh
    find -name '*.sh' \
        -exec sed -i -e 's|#!/bin/bash|#!/bin/bash|' {} +
else
    print_usage
fi
