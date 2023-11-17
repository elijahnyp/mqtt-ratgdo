#!/bin/bash
echo Building $1 Firmware
upper_model=`echo $1| tr '[:lower:]' '[:upper:]'`
lower_model=`echo $1| tr '[:upper:]' '[:lower:]'`

# version update
# version.txt file
mv version.txt version.build.bak
# cp include/settings.h include/settings.build.bak
# cp main/CMakeLists.txt main/CMakeLists.build.bak
echo $lower_model $(git rev-parse --short HEAD) > version.txt

if [[ ! -d ./out ]]; then
    mkdir out
fi
built=0
idf.py build
if [[ $? == 0 ]]; then
    cp build/ratgdo.bin out/ratgdo.bin
    built=1
else
    echo "build of ratgdo failed"
    built=0
fi

if [[ $built == 0 ]]; then
    exit 1
else
    exit 0
fi