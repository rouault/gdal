#!/bin/sh

set -e

export PROJ_NETWORK=ON

echo 'Running CPP unit tests'
(cd build && make quicktest)

echo 'Running Python unit tests'
# install test dependencies
sudo -H pip3 install -r autotest/requirements.txt

NPROC=$(sysctl -n hw.ncpu)
echo "NPROC=${NPROC}"

export DYLD_LIBRARY_PATH=$PWD/build

echo "otool -L build/libgdal.dylib"
otool -L build/libgdal.dylib

echo "otool -L build/swig/python/osgeo/_gdal.cpython*"
otool -L build/swig/python/osgeo/_gdal.cpython*

export PYTHONPATH=$PWD/build/swig/python
python3 -c "from osgeo import gdal"

# Run all the Python autotests
# FIXME: disabled for now because of https://github.com/OSGeo/gdal/issues/9723
#(cd build && ctest -V -R autotest -j${NPROC})
