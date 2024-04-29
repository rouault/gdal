/******************************************************************************
 *
 * Project:  Viewshed Generator
 * Purpose:  Viewshed Generator mainline.
 * Author:   Tamas Szekeres <szekerest@gmail.com>
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

#include "cpl_conv.h"
#include "cpl_string.h"
#include "gdal_version.h"
#include "gdal.h"
#include "gdal_alg.h"
#include "gdal_priv.h"
#include "ogr_api.h"
#include "ogr_srs_api.h"
#include "ogr_spatialref.h"
#include "commonutils.h"
#include "gdalargumentparser.h"

#include "viewshed.h"

/************************************************************************/
/*                                main()                                */
/************************************************************************/

//ABELL
//MAIN_START(argc, argv)
int main(int argc, char **argv)

{
    using namespace gdal;

    EarlySetConfigOptions(argc, argv);

    GDALAllRegister();

    argc = GDALGeneralCmdLineProcessor(argc, &argv, 0);
    CPLStringList aosArgv;
    aosArgv.Assign(argv, /* bTakeOwnership= */ true);
    if (argc < 1)
        std::exit(-argc);

    GDALArgumentParser argParser(aosArgv[0], /* bForBinary=*/true);

    argParser.add_description(
        _("Calculates a viewshed raster from an input raster DEM."));

    argParser.add_epilog(_("For more details, consult "
                           "https://gdal.org/programs/gdal_viewshed.html"));

    Viewshed::Options o;

    argParser.add_output_format_argument(o.outputFormat);
    argParser.add_argument("-ox")
        .store_into(o.observer.x)
        .required()
        .metavar("<value>")
        .help(_("The X position of the observer (in SRS units)."));

    argParser.add_argument("-oy")
        .store_into(o.observer.y)
        .required()
        .metavar("<value>")
        .help(_("The Y position of the observer (in SRS units)."));

    argParser.add_argument("-oz")
        .default_value(2)
        .store_into(o.observer.z)
        .metavar("<value>")
        .nargs(1)
        .help(_("The height of the observer above the DEM surface in the "
                "height unit of the DEM."));

    argParser.add_argument("-vv")
        .default_value(255)
        .store_into(o.visibleVal)
        .metavar("<value>")
        .nargs(1)
        .help(_("Pixel value to set for visible areas."));

    argParser.add_argument("-iv")
        .default_value(0)
        .store_into(o.invisibleVal)
        .metavar("<value>")
        .nargs(1)
        .help(_("Pixel value to set for invisible areas."));

    argParser.add_argument("-ov")
        .default_value(0)
        .store_into(o.outOfRangeVal)
        .metavar("<value>")
        .nargs(1)
        .help(
            _("Pixel value to set for the cells that fall outside of the range "
              "specified by the observer location and the maximum distance."));

    argParser.add_creation_options_argument(o.creationOpts);

    argParser.add_argument("-a_nodata")
        .default_value(-1.0)
        .store_into(o.nodataVal)
        .metavar("<value>")
        .nargs(1)
        .help(_("The value to be set for the cells in the output raster that "
                "have no data."));

    argParser.add_argument("-tz")
        .default_value(0.0)
        .store_into(o.targetHeight)
        .metavar("<value>")
        .nargs(1)
        .help(_("The height of the target above the DEM surface in the height "
                "unit of the DEM."));

    argParser.add_argument("-md")
        .default_value(0)
        .store_into(o.maxDistance)
        .metavar("<value>")
        .nargs(1)
        .help(_("Maximum distance from observer to compute visibility."));

    // Value for standard atmospheric refraction. See
    // doc/source/programs/gdal_viewshed.rst
    argParser.add_argument("-cc")
        .default_value(0.85714)
        .store_into(o.curveCoeff)
        .metavar("<value>")
        .nargs(1)
        .help(_("Coefficient to consider the effect of the curvature and "
                "refraction."));

    int nBandIn = 1;
    argParser.add_argument("-b")
        .default_value(nBandIn)
        .store_into(nBandIn)
        .metavar("<value>")
        .nargs(1)
        .help(_("Select an input band band containing the DEM data."));

    argParser.add_argument("-om")
        .default_value("NORMAL")
        .choices("NORMAL", "DEM", "GROUND")
        .metavar("NORMAL|DEM|GROUND")
        .action([&into = o.outputMode](const std::string& value)
            {
                if (EQUAL(value.c_str(), "DEM"))
                    into = Viewshed::OutputMode::DEM;
                else if (EQUAL(value.c_str(), "GROUND"))
                    into = Viewshed::OutputMode::Ground;
                else
                    into = Viewshed::OutputMode::Normal;
            })
        .nargs(1)
        .help(_("Sets what information the output contains."));

    bool bQuiet = false;
    argParser.add_quiet_argument(&bQuiet);

    std::string osSrcFilename;
    argParser.add_argument("src_filename")
        .store_into(osSrcFilename)
        .metavar("<src_filename>");

    argParser.add_argument("dst_filename")
        .store_into(o.outputFilename)
        .metavar("<dst_filename>");

    try
    {
        argParser.parse_args(aosArgv);
    }
    catch (const std::exception &err)
    {
        argParser.display_error_and_usage(err);
        std::exit(1);
    }

    if (o.maxDistance < 0)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Max distance must be non-negative.");
        exit(2);
    }

    if (o.outputFormat.empty())
    {
        o.outputFormat = GetOutputDriverForRaster(o.outputFilename.c_str());
        if (o.outputFormat.empty())
        {
            exit(2);
        }
    }

    // For double values that are out of range for byte raster output,
    // set to zero.
    if (o.outputMode == Viewshed::OutputMode::Normal)
    {
        // Values less than zero are sentinel to not have nodata.
        if (o.nodataVal > std::numeric_limits<uint8_t>::max())
            o.nodataVal = 0;

        if (o.outOfRangeVal < 0 || o.outOfRangeVal > std::numeric_limits<uint8_t>::max())
            o.outOfRangeVal = 0;
    }

    /* -------------------------------------------------------------------- */
    /*      Open source raster file.                                        */
    /* -------------------------------------------------------------------- */
    GDALDatasetH hSrcDS = GDALOpen(osSrcFilename.c_str(), GA_ReadOnly);
    if (hSrcDS == nullptr)
        exit(2);

    GDALRasterBandH hBand = GDALGetRasterBand(hSrcDS, nBandIn);
    if (hBand == nullptr)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Band %d does not exist on dataset.", nBandIn);
        exit(2);
    }

    if (!argParser.is_used("-cc"))
    {
        const OGRSpatialReference *poSRS =
            GDALDataset::FromHandle(hSrcDS)->GetSpatialRef();
        if (poSRS)
        {
            OGRErr eSRSerr;
            const double dfSemiMajor = poSRS->GetSemiMajor(&eSRSerr);
            if (eSRSerr != OGRERR_FAILURE &&
                fabs(dfSemiMajor - SRS_WGS84_SEMIMAJOR) >
                    0.05 * SRS_WGS84_SEMIMAJOR)
            {
                o.curveCoeff = 1.0;
                CPLDebug("gdal_viewshed",
                         "Using -cc=1.0 as a non-Earth CRS has been detected");
            }
        }
    }

    /* -------------------------------------------------------------------- */
    /*      Invoke.                                                         */
    /* -------------------------------------------------------------------- */
    Viewshed oViewshed(o);

    bool bSuccess = oViewshed.run(hBand, bQuiet ? GDALDummyProgress : GDALTermProgress);

    GDALDatasetH hDstDS = GDALDataset::FromHandle(oViewshed.output().release());

    /**
    GDALDatasetH hDstDS = GDALViewshedGenerate(
        hBand, osFormat.c_str(), osDstFilename.c_str(),
        aosCreationOptions.List(), dfObserverX, dfObserverY, dfObserverHeight,
        dfTargetHeight, dfVisibleVal, dfInvisibleVal, dfOutOfRangeVal,
        dfNoDataVal, dfCurvCoeff, GVM_Edge, dfMaxDistance, pfnProgress, nullptr,
        outputMode, nullptr);
    **/

    GDALClose(hSrcDS);
    if (GDALClose(hDstDS) != CE_None)
        bSuccess = false;

    GDALDestroyDriverManager();
    OGRCleanupAll();

    return bSuccess ? 0 : 1;
}

//ABELL
//MAIN_END
