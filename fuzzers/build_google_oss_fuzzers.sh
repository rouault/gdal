#!/bin/bash
# WARNING: this script is used by https://github.com/google/oss-fuzz/blob/master/projects/gdal/build.sh
# and should not be renamed or moved without updating the above

set -e

if [ "$SRC" == "" ]; then
    echo "SRC env var not defined"
    exit 1
fi

if [ "$OUT" == "" ]; then
    echo "OUT env var not defined"
    exit 1
fi

if [ "$CXX" == "" ]; then
    echo "CXX env var not defined"
    exit 1
fi

if [ "$LIB_FUZZING_ENGINE" = "" ]; then
    export LIB_FUZZING_ENGINE=-lFuzzingEngine
fi

SRC_DIR=$(dirname $0)/..

if [ "$LIBGDAL" = "" ]; then
  LIBGDAL="$SRC_DIR/build/libgdal.a"
fi

build_fuzzer()
{
    fuzzerName=$1
    sourceFilename=$2
    shift
    shift
    echo "Building fuzzer $fuzzerName"
    if test -d $SRC/install/lib; then
        $CXX $CXXFLAGS -std=c++17 -I$SRC_DIR/port -I$SRC_DIR/build/port -I$SRC_DIR/gcore -I$SRC_DIR/build/gcore -I$SRC_DIR/alg -I$SRC_DIR/apps -I$SRC_DIR/ogr -I$SRC_DIR/ogr/ogrsf_frmts -I$SRC_DIR/ogr/ogrsf_frmts/sqlite \
            $sourceFilename "$@" -o $OUT/$fuzzerName \
            $LIB_FUZZING_ENGINE $LIBGDAL $EXTRA_LIBS $SRC/install/lib/*.a
    else
        $CXX $CXXFLAGS -std=c++17 -I$SRC_DIR/port -I$SRC_DIR/build/port -I$SRC_DIR/gcore -I$SRC_DIR/build/gcore -I$SRC_DIR/alg -I$SRC_DIR/apps -I$SRC_DIR/ogr -I$SRC_DIR/ogr/ogrsf_frmts -I$SRC_DIR/ogr/ogrsf_frmts/sqlite \
            $sourceFilename "$@" -o $OUT/$fuzzerName \
            $LIB_FUZZING_ENGINE $LIBGDAL $EXTRA_LIBS
    fi
}

build_ogr_specialized_fuzzer()
{
    format=$1
    registerFunc=$2
    memFilename=$3
    gdalFilename=$4
    fuzzerName="${format}_fuzzer"
    build_fuzzer $fuzzerName $(dirname $0)/ogr_fuzzer.cpp -DREGISTER_FUNC=$registerFunc -DMEM_FILENAME="\"$memFilename\"" -DGDAL_FILENAME="\"$gdalFilename\"" -DOGR_SKIP="\"CAD\""
}

build_gdal_specialized_fuzzer()
{
    format=$1
    registerFunc=$2
    memFilename=$3
    gdalFilename=$4
    fuzzerName="${format}_fuzzer"
    build_fuzzer $fuzzerName $(dirname $0)/gdal_fuzzer.cpp -DREGISTER_FUNC=$registerFunc -DMEM_FILENAME="\"$memFilename\"" -DGDAL_FILENAME="\"$gdalFilename\""
}

build_ogr_specialized_fuzzer parquet RegisterOGRParquet "/vsimem/test.parquet" "/vsimem/test.parquet"

echo "[libfuzzer]" > $OUT/wkb_import_fuzzer.options
echo "max_len = 100000" >> $OUT/wkb_import_fuzzer.options

echo "[libfuzzer]" > $OUT/wkt_import_fuzzer.options
echo "max_len = 100000" >> $OUT/wkt_import_fuzzer.options

echo "[libfuzzer]" > $OUT/gml_geom_import_fuzzer.options
echo "max_len = 100000" >> $OUT/gml_geom_import_fuzzer.options

echo "[libfuzzer]" > $OUT/spatialite_geom_import_fuzzer.options
echo "max_len = 100000" >> $OUT/spatialite_geom_import_fuzzer.options

echo "[libfuzzer]" > $OUT/osr_set_from_user_input_fuzzer.options
echo "max_len = 10000" >> $OUT/osr_set_from_user_input_fuzzer.options
