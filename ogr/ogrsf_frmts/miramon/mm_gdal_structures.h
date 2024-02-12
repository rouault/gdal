#ifndef __MM_GDAL_STRUCTURES_H
#define __MM_GDAL_STRUCTURES_H
/* -------------------------------------------------------------------- */
/*      Constants used in GDAL and in MiraMon                           */
/* -------------------------------------------------------------------- */
#ifdef GDAL_COMPILATION
#include "cpl_conv.h"  // For FILE_TYPE
#include "mm_gdal_constants.h"
#else
#include "F64_str.h"  // For FILE_64
#include "mm_gdal\mm_gdal_constants.h"
#endif

#include "mm_constants.h"

#ifdef GDAL_COMPILATION
CPL_C_START  // Necessary for compiling in GDAL project
#endif

#ifdef GDAL_COMPILATION
#define FILE_TYPE VSILFILE
#else
#define FILE_TYPE FILE_64
#endif

    /* Internal field of a extended DBF. It's a copy of MiraMon internal 
    structure but translated to be understood by anyone who wants to 
    review the code of the driver
*/

    struct MM_CAMP  // FIELD
{
    // Name of the field
    char NomCamp[MM_MAX_LON_FIELD_NAME_DBF];

    // Name of the field in dBASEIII
    char NomCampDBFClassica[MM_MAX_LON_CLASSICAL_FIELD_NAME_DBF];

    // Type of the field C, N, D, L, M, F, G and B
    char TipusDeCamp;
    MM_BOOLEAN Is64;  // Is an signed 64 bit integer

    // Number of decimals if it's a float
    MM_BYTE DecimalsSiEsFloat;

    // Number of bytes of a field
    MM_TIPUS_BYTES_PER_CAMP_DBF BytesPerCamp;

    // Acumulated bytes before a field
    MM_TIPUS_BYTES_ACUMULATS_DBF BytesAcumulats;

    // Not used in GDAL
    char *separador[MM_NUM_IDIOMES_MD_MULTIDIOMA];

    // Description of the field
    char DescripcioCamp[MM_NUM_IDIOMES_MD_MULTIDIOMA]
                       [MM_MAX_LON_DESCRIPCIO_CAMP_DBF];

    MM_BYTE AmpleDesitjat;
    MM_BYTE AmpleDesitjatOriginal;

    MM_BYTE reservat_1[MM_MAX_LON_RESERVAT_1_CAMP_BD_XP];

    MM_BYTE reservat_2[MM_MAX_LON_RESERVAT_2_CAMP_BD_XP];
    MM_BYTE MDX_camp_flag;
    MM_BYTE TipusCampGeoTopo;
};

struct MM_BASE_DADES_XP  // MiraMon Database Structure
{
    // Extended DBF file name
    char szNomFitxer[MM_CPL_PATH_BUF_SIZE];

    // Temporal table
    MM_BOOLEAN EsTaulaTemporal;

    FILE_TYPE *pfBaseDades;

    // Charset of the DBF
    MM_BYTE JocCaracters;
    //BYTE InfoJocCaracExterna;
    char ModeLectura[4];
    MM_EXT_DBF_N_RECORDS nRecords;
    MM_TIPUS_BYTES_ACUMULATS_DBF BytesPerFitxa;
    MM_EXT_DBF_N_FIELDS ncamps;
    struct MM_CAMP *Camp;
    MM_FIRST_RECORD_OFFSET_TYPE OffsetPrimeraFitxa;
    MM_EXT_DBF_N_FIELDS CampIdGrafic;
    MM_EXT_DBF_N_FIELDS CampIdEntitat;
    short int any;
    MM_BYTE mes;
    MM_BYTE dia;

    MM_BYTE versio_dbf;

    MM_BYTE reservat_1[MM_MAX_LON_RESERVAT_1_BASE_DADES_XP];
    MM_BYTE transaction_flag;
    MM_BYTE encryption_flag;
    MM_BYTE dbf_on_a_LAN[MM_MAX_LON_DBF_ON_A_LAN_BASE_DADES_XP];
    MM_BYTE MDX_flag;
    MM_BYTE reservat_2[MM_MAX_LON_RESERVAT_2_BASE_DADES_XP];
};
#ifdef GDAL_COMPILATION
CPL_C_END  // Necessary for compiling in GDAL project
#endif
#endif  //__MM_GDAL_STRUCTURES_H
