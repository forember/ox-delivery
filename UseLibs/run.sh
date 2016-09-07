#!/bin/bash

cd "$(dirname "$0")"

./clean.sh

qmake -project
qmake *.pro
cat libs.txt >> "$(find -name "*.pro")"
echo "CONFIG+=debug" >> "$(find -name "*.pro")"
qmake -o Makefile *.pro

#export LD_LIBRARY_PATH=/usr/local/lib/KCPP

#make 2>&1 >/dev/null | less
make 2>&1 #| less

echo 'Configuring linker. Requires root.'
sudo ldconfig
