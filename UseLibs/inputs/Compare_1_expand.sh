#!/bin/sh

cd "$(dirname "$0")"

cp Compare_1.png Compare_1_expanded.png
cp Compare_1.wf Compare_1_expanded.wf
python Compare_1_expand.py
