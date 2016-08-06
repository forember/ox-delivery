#!/bin/sh

print_usage () {
    echo 'usage: convert_install_path.sh --local'
    echo
    echo '      Convert install path to /usr/local/'
    echo
    echo 'usage: convert_install_path.sh --usr'
    echo
    echo '      Convert install path to /usr/'
    exit 1
}

if [ -z "$1" ]; then
    print_usage
fi

cd "$(dirname "$0")"

if [ "$1" = '--local' ]; then
    ./clean_all.sh
    find -type f ! -name convert_install_path.sh \
        -exec sed -i -e 's|/usr/lib/|/usr/local/lib/|g' {} +
    find -type f ! -name convert_install_path.sh \
        -exec sed -i -e 's|/usr/include/|/usr/local/include/|g' {} +
    find -type f ! -name convert_install_path.sh \
        -exec sed -i -e 's|/usr/local/lib/cmake/|/usr/lib/cmake/|g' {} +
elif [ "$1" = '--usr' ]; then
    ./clean_all.sh
    find -type f ! -name convert_install_path.sh \
        -exec sed -i -e 's|/usr/local/lib/|/usr/lib/|g' {} +
    find -type f ! -name convert_install_path.sh \
        -exec sed -i -e 's|/usr/local/include/|/usr/include/|g' {} +
else
    print_usage
fi
