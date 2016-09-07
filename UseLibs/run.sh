#!/bin/bash

cd "$(dirname "$0")"

showlogs () {
    bash_retval=${PIPESTATUS[0]}
    if [ "$bash_retval" = '' -o "$bash_retval" -ne 0 ]; then
        for file in "$@"; do
            echo ".${file}.log:"
            cat -n ".${file}.log"
            echo
        done | less
        rm -f .clean.log
        exit $bash_retval
    fi
}

./clean.sh 2>&1 | tee .clean.log

qmake -project 2>&1 | tee .qmake1.log
showlogs clean qmake1
qmake *.pro 2>&1 | tee .qmake2.log
showlogs clean qmake1 qmake2
cat libs.txt >> "$(find -name "*.pro")"
qmake -o Makefile *.pro 2>&1 | tee .qmake3.log
showlogs clean qmake1 qmake2 qmake3

#export LD_LIBRARY_PATH=/usr/local/lib/KCPP

#make 2>&1 >/dev/null | less
make 2>&1 | tee .make.log
showlogs clean qmake1 qmake2 qmake3 make

echo 'Configuring linker. Requires root.'
sudo ldconfig 2>&1 | tee .ldconfig.log
showlogs clean qmake1 qmake2 qmake3 make ldconfig
rm -f .clean.log
