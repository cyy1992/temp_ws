#!/bin/bash

version=$1
if [  -d "build" ]; then
	rm -r build
fi

mkdir build && cd build && mkdir -p apriltag/usr/local
cmake -DCMAKE_INSTALL_PREFIX=apriltag/usr/local .. && make install -j4

cp -r ../DEBIAN apriltag/


cd apriltag/DEBIAN/
cat control | sed "s/{VERSION}/${version}/g" >control || (echo "sed error" && exit 6 )
exit 6
cd ..
chmod -R +x DEBIAN
cd ..
dpkg -b apriltag apriltag_${version}.deb

