/******************************************************************************
 *
 * Project:  Viewshed Generation
 * Purpose:  Core algorithm implementation for viewshed generation.
 * Author:   Tamas Szekeres, szekerest@gmail.com
 *
 ******************************************************************************
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#include <cstdint>
#include <memory>
#include <string>

#include "cpl_progress.h"
#include "gdal_priv.h"

namespace gdal
{

class Viewshed
{
  public:
    enum class OutputMode
    {
        Normal,
        DEM,
        Ground
    };

    enum class CellMode
    {
        Diagonal,
        Edge,
        Max,
        Min
    };

    struct Point
    {
        double x;
        double y;
        double z;
    };

    struct Options
    {
        Point observer{0, 0, 0};
        uint8_t visibleVal{0};
        uint8_t invisibleVal{0};
        uint8_t outOfRangeVal{0};
        double nodataVal{0.0};
        double targetHeight{0.0};
        double maxDistance{0.0};
        double curveCoeff{0.0};
        OutputMode outputMode{OutputMode::Normal};
        std::string outputFormat{};
        std::string outputFilename{};
        CPLStringList creationOpts{};
        CellMode cellMode{CellMode::Edge};
    };

    CPL_DLL explicit Viewshed(const Options &opts) : oOpts{opts}, poDstDS{}
    {
    }

    CPL_DLL bool run(GDALRasterBandH hBand, GDALProgressFunc pfnProgress);

    CPL_DLL std::unique_ptr<GDALDataset> output()
    {
        return std::move(poDstDS);
    }

  private:
    Options oOpts;
    std::unique_ptr<GDALDataset> poDstDS;
};

}  // namespace gdal
