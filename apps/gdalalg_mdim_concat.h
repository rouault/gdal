/******************************************************************************
 *
 * Project:  GDAL
 * Purpose:  gdal "mdim concat" subcommand
 * Author:   Even Rouault <even dot rouault at spatialys.com>
 *
 ******************************************************************************
 * Copyright (c) 2025, Even Rouault <even dot rouault at spatialys.com>
 *
 * SPDX-License-Identifier: MIT
 ****************************************************************************/

#ifndef GDALALG_MDIM_CONCAT_INCLUDED
#define GDALALG_MDIM_CONCAT_INCLUDED

#include "gdalalgorithm.h"

#include "gdal_multidim.h"

#include <optional>
#include <utility>

//! @cond Doxygen_Suppress

/************************************************************************/
/*                       GDALMdimConcatAlgorithm                        */
/************************************************************************/

class GDALMdimConcatAlgorithm final : public GDALAlgorithm
{
  public:
    static constexpr const char *NAME = "concat";
    static constexpr const char *DESCRIPTION =
        "Concatenate multidimensional datasets.";
    static constexpr const char *HELP_URL = "/programs/gdal_mdim_concat.html";

    explicit GDALMdimConcatAlgorithm();

  private:
    bool RunImpl(GDALProgressFunc pfnProgress, void *pProgressData) override;

    std::string m_outputFormat{};
    std::vector<GDALArgDatasetValue> m_inputDatasets{};
    std::vector<std::string> m_openOptions{};
    std::vector<std::string> m_inputFormats{};
    GDALArgDatasetValue m_outputDataset{};
    std::vector<std::string> m_creationOptions{};
    bool m_overwrite = false;
    std::string m_array{};

    // Describes a dimension of the concatenated array.
    struct DimensionDesc
    {
        std::string osName{};
        std::string osType{};
        std::string osDirection{};
        uint64_t nSize = 0;

        // Used for dimensions with irregular spaced labels
        int nProgressionSign =
            0;  // 1=increasing, -1=decreasing, 0=single value
        // Groups of irrugularly spaced values. In common cases,
        // aaValues[i].size() will be just one
        std::vector<std::vector<double>> aaValues{};

        // Used for dimensions with regularly spaced labels
        double dfStart = 0;
        double dfIncrement = 0;
    };

    // Minimum information about a dimension of a source array.
    struct SourceShortDimDesc
    {
        uint64_t nSize = 0;
        double dfStart = 0;
        bool bIsRegularlySpaced = false;
    };

    std::optional<DimensionDesc>
    // cppcheck-suppress functionStatic
    GetDimensionDesc(const std::string &osDSName,
                     const std::shared_ptr<GDALDimension> &poDim) const;

    std::optional<std::vector<DimensionDesc>> BuildDimensionDesc(
        std::shared_ptr<GDALMDArray> &poFirstSourceArray,
        std::vector<std::vector<SourceShortDimDesc>> &aaoSourceShortDimDesc);
};

//! @endcond

#endif
