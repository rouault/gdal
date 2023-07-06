#!/bin/sh

set -eu

export CXXFLAGS="--coverage"
export CFLAGS="--coverage"
export LDFLAGS="-lgcov"

cmake ${GDAL_SOURCE_DIR:=..} \
    -DUSE_CCACHE=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DGDAL_USE_TIFF_INTERNAL=ON \
    -DGDAL_USE_GEOTIFF_INTERNAL=ON \
    -DGDAL_BUILD_OPTIONAL_DRIVERS=OFF -DOGR_BUILD_OPTIONAL_DRIVERS=OFF

make -j$(nproc)
