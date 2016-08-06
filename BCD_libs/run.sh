#!/bin/sh

cd "$(dirname "$0")"

./clean.sh

cmake .
make 2>&1 >/dev/null #| less
echo 'Installing and configuring linker. Requires root.'
sudo make install
sudo ldconfig
