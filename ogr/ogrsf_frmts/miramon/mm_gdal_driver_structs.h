#ifndef __MM_GDAL_DRIVER_STRUCTS_H
#define __MM_GDAL_DRIVER_STRUCTS_H
/* -------------------------------------------------------------------- */
/*      Necessary functions to read/write a MiraMon Vector File         */
/* -------------------------------------------------------------------- */


#ifdef GDAL_COMPILATION
#include "mm_gdal_constants.h"
#include "mm_gdal_structures.h"
CPL_C_START // Necessary for compiling in GDAL project
#else
#include "mm_gdal\mm_gdal_constants.h"
#include "mm_gdal\mm_gdal_structures.h"
// Falta aix� o es queixa a diversos llocs
#include "str_snyd.h"	// Per a struct SNY_TRANSFORMADOR_GEODESIA  
#endif

// For MetaData
#define SECTION_VERSIO         "VERSIO"
#define KEY_Vers               "Vers"
#define KEY_SubVers            "SubVers"
#define MM_VERS                4
#define MM_SUBVERS             3
#define KEY_VersMetaDades      "VersMetaDades"
#define KEY_SubVersMetaDades   "SubVersMetaDades"
#define MM_VERS_METADADES      5
#define MM_SUBVERS_METADADES   0
#define SECTION_METADADES      "METADADES"
#define KEY_FileIdentifier     "FileIdentifier"
#define SECTION_IDENTIFICATION "IDENTIFICATION"
#define KEY_code               "code"
#define KEY_codeSpace          "codeSpace"
#define KEY_DatasetTitle       "DatasetTitle"
#define SECTION_OVERVIEW        "OVERVIEW"
#define SECTION_OVVW_ASPECTES_TECNICS "OVERVIEW:ASPECTES_TECNICS"
#define KEY_ArcSource           "ArcSource"
#define SECTION_EXTENT          "EXTENT"
#define KEY_toler_env           "toler_env"
#define KEY_MinX                "MinX"
#define KEY_MaxX                "MaxX"
#define KEY_MinY                "MinY"
#define KEY_MaxY                "MaxY"
#define KEY_CreationDate        "CreationDate"
#define SECTION_SPATIAL_REFERENCE_SYSTEM "SPATIAL_REFERENCE_SYSTEM"
#define SECTION_HORIZONTAL      "HORIZONTAL"
#define KEY_HorizontalSystemIdentifier "HorizontalSystemIdentifier"
#define SECTION_TAULA_PRINCIPAL "TAULA_PRINCIPAL"
#define KEY_IdGrafic            "IdGrafic"
#define KEY_TipusRelacio        "TipusRelacio"
#define KEY_descriptor          "descriptor"
#define KEY_HorizontalSystemDefinition "HorizontalSystemDefinition"
#define KEY_unitats             "unitats"
#define KEY_unitatsY            "unitatsY"
#define KEY_language            "language" 
#define KEY_Value_eng           "eng" 
#define KEY_MDIdiom             "MDIdiom" 
#define KEY_characterSet        "characterSet" 
#define KEY_Value_characterSet  "006" 


// Types of layers in MiraMon
#define MM_LayerType_Unknown    0 // Unknown type
#define MM_LayerType_Point      1 // Layer of Points
#define MM_LayerType_Point3d    2 // Layer of 3D Points
#define MM_LayerType_Arc        3 // Layer of Arcs
#define MM_LayerType_Arc3d      4 // Layer of 3D Arcs
#define MM_LayerType_Pol        5 // Layer of Polygons
#define MM_LayerType_Pol3d      6 // Layer of 3D Polygons
#define MM_LayerType_Node       7 // Layer of Nodes (internal)
#define MM_LayerType_Raster     8 // Layer of Raster Type

#define MM_FIRST_NUMBER_OF_POINTS 10000
#define MM_INCR_NUMBER_OF_POINTS    1000
#define MM_FIRST_NUMBER_OF_ARCS     10000
#define MM_INCR_NUMBER_OF_ARCS      1000
#define MM_FIRST_NUMBER_OF_NODES     20000  // 2*MM_FIRST_NUMBER_OF_ARCS
#define MM_INCR_NUMBER_OF_NODES      2000
#define MM_FIRST_NUMBER_OF_POLYGONS 10000
#define MM_INCR_NUMBER_OF_POLYGONS  1000
#define MM_FIRST_NUMBER_OF_VERTICES     10000
#define MM_INCR_NUMBER_OF_VERTICES      1000

#define MM_250MB   262144000
#define MM_500MB   524288000

#define MM_INTERNAL_FID     unsigned __int64
#define MM_EXT_DBF_N_FIELDS unsigned __int32

// Version asked for user
#define MM_UNKNOWN_VERSION    0
#define MM_LAST_VERSION       1
#define MM_32BITS_VERSION     2
#define MM_64BITS_VERSION     3

// AddFeature returns
#define MM_CONTINUE_WRITING_FEATURES        0
#define MM_FATAL_ERROR_WRITING_FEATURES     1
#define MM_STOP_WRITING_FEATURES            2

// Size of the FID (and OFFSETS) in the current version
#define MM_SIZE_OF_FID_4BYTES_VERSION   4
#define MM_SIZE_OF_FID_8BYTES_VERSION   8


/*  Different values that first member of every PAL section element can take*/
#define MM_EXTERIOR_ARC_SIDE    0x01
#define MM_END_ARC_IN_RING      0x02
#define MM_ROTATE_ARC           0x04

#define ARC_VRT_INICI  0
#define ARC_VRT_FI     1

#define STATISTICAL_UNDEF_VALUE (2.9E+301)

#define MAXIMUM_OBJECT_INDEX_IN_2GB_VECTORS _UI32_MAX 
#define MAXIMUM_OFFSET_IN_2GB_VECTORS _UI32_MAX

// Number of rings a polygon could have (it's just an aproximation)
#define MM_MEAN_NUMBER_OF_RINGS 10  

// Number of coordinates a feature could have (it's just an aproximation)
#define MM_MEAN_NUMBER_OF_NCOORDS 100
#define MM_MEAN_NUMBER_OF_COORDS 1000  

// Number of fields that the suppose to have.
#define MM_INIT_NUMBER_OF_RECORDS   1
#define MM_INC_NUMBER_OF_RECORDS   5
#define MM_INIT_NUMBER_OF_FIELDS    20
#define MM_INC_NUMBER_OF_FIELDS    10

enum FieldType
{
  /*! Numeric Field                                         */ MM_Numeric=0,
  /*! Character Fi eld                                      */ MM_Character=1,
  /*! Data Field                                            */ MM_Data=2,
  /*! Logic Field                                           */ MM_Logic=3
};


// Size of disk parts of the MiraMon vectorial format
// Common header
#define MM_HEADER_SIZE_32_BITS  48
#define MM_HEADER_SIZE_64_BITS  64

// Points
#define MM_SIZE_OF_TL           16

// Nodes
#define MM_SIZE_OF_NH_32BITS    8
#define MM_SIZE_OF_NH_64BITS    12
#define MM_SIZE_OF_NL_32BITS    4
#define MM_SIZE_OF_NL_64BITS    8

// Arcs
#define MM_SIZE_OF_AH_32BITS    56
#define MM_SIZE_OF_AH_64BITS    72
#define MM_SIZE_OF_AL           16

// Polygons
#define MM_SIZE_OF_PS_32BITS    8
#define MM_SIZE_OF_PS_64BITS    16
#define MM_SIZE_OF_PH_32BITS    64
#define MM_SIZE_OF_PH_64BITS    80
#define MM_SIZE_OF_PAL_32BITS   5
#define MM_SIZE_OF_PAL_64BITS   9

// 3D part
#define MM_SIZE_OF_ZH           32
#define MM_SIZE_OF_ZD_32_BITS   24
#define MM_SIZE_OF_ZD_64_BITS   32

/* -------------------------------------------------------------------- */
/*      Structures                                                      */
/* -------------------------------------------------------------------- */
// Auxiliary structures
struct MMBoundingBox
{
    double dfMinX;
    double dfMaxX;
    double dfMinY;
    double dfMaxY;
};

struct MM_POINT_2D
{
    double dfX;
    double dfY;
};

struct ARC_VRT_STRUCTURE
{
	struct MM_POINT_2D vertice;   
	MM_BOOLEAN bIniFi; // boolean:  0=inicial, 1=final
	MM_INTERNAL_FID nIArc;  // Internal arc index 
	MM_INTERNAL_FID nINod;	// Internal node index, empty at the beginning */
};

struct MM_FLUSH_INFO
{
    __int32 nMyDiskSize;
    unsigned __int64 NTimesFlushed;

    // Pointer to an OPEN file where to flush.
    FILE_TYPE *pF; 
    // Offset in the disk where to flush 
    MM_FILE_OFFSET OffsetWhereToFlush;
    
    unsigned __int64 TotalSavedBytes;  // Internal use


    // Block where to be saved
    unsigned __int64 SizeOfBlockToBeSaved;
    void *pBlockToBeSaved;

    // Block where to save the pBlockToBeSaved
    void *pBlockWhereToSave;
    // Number of full bytes: flushed every time it's needed
    unsigned __int64 nNumBytes;
    // Number of bytes allocated: flushed every time it's needed
    unsigned __int64 nBlockSize;

    // Internal Use
    MM_FILE_OFFSET CurrentOffset;
};

// MIRAMON METADATA
struct MiraMonVectorMetaData
{
    char aLayerName[MM_MAX_LEN_LAYER_NAME];
    char aArcFile[MM_MAX_LEN_LAYER_NAME]; // Polygon's arc name
    int ePlainLT; // Plain layer type (no 3D specified): MM_LayerType_Point, 
                // MM_LayerType_Arc, MM_LayerType_Node, MM_LayerType_Pol
	char *pSRS; // EPSG code of the coordinate system information.
    char *pXUnit; // X units if pszSRS is empty. 
    char *pYUnit; // Y units if pszSRS is empty. If Y units is empty,
                  // X unit will be assigned as Y unit by default.

	struct MMBoundingBox hBB; // Bounding box of the entire layer

    // Pointer to a Layer DataBase, used to create MiraMon DBF (extended) file.
    struct MiraMonDataBase *pLayerDB;

};

struct MiraMonRasterMetaData
{
    char aLayerName[MM_MAX_LEN_LAYER_NAME];
    char *pSRS; // EPSG code of the coordinate system information.
    char *pXUnit; // X units if pszSRS is empty. 
    char *pYUnit; // Y units if pszSRS is empty. If Y units is empty,
                  // X unit will be assigned as Y unit by default.

	unsigned __int32 nBands; // number of bands (in vector layer, always 1).

    struct MMBoundingBox *hBB; // Bounding box of every band of a raster file
                               // or bounding box of the entire layer in case
                               // of a vector file.

    // Only for rasters, for each band:
    char **pBandName;
    unsigned __int32 *pnBandNumber;
    char **pBandDescription;
    unsigned __int64 *nNCol;
    unsigned __int64 *nNFil;
    double *pXResolution;
    double *pYResolution;
    enum DataType *peDataType;
    enum TreatmentVariable *peTreatmentVariable;
    char **pValueUnit;
    MM_BOOLEAN *pHasANoDataVaule;
    double *pNoDataValue;

	/*
    �$� Metadata que s'ha de calcular/revisar mentre s'afegeixen features:    
        Nom fitxer de sortida per cada banda
        M�nim i m�xim dels valors dels p�xels
        DonaMD_CoverageContentType? (revisar GDALMM2.c sobre la l�nia 632
            
            */
};

// MIRAMON DATA BASE
#define MM_GRAPHICAL_ID_INIT_SIZE   5
#define MM_N_VERTEXS_INIT_SIZE      12
#define MM_LONG_ARC_INIT_SIZE       12
#define MM_LONG_ARC_DECIMALS_SIZE   6
#define MM_NODE_INI_INIT_SIZE       5
#define MM_NODE_FI_INIT_SIZE        5

#define MM_PERIMETRE_INIT_SIZE      13
#define MM_PERIMETRE_DECIMALS_SIZE  6
#define MM_AREA_INIT_SIZE           14
#define MM_AREA_DECIMALS_SIZE       6

#define MM_N_ARCS_INIT_SIZE         3
#define MM_N_ARCS_DECIMALS_SIZE     3

#define MM_ARCS_A_NOD_INIT_SIZE     1


struct MiraMonFieldValue
{
    MM_BOOLEAN bIsValid;   // If 1 the value is filled. If 0, there is no value.
    #define MM_INIT_STRING_FIELD_VALUE   50000  // Never less than 10
	MM_NUMERATOR_DBF_FIELD_TYPE nNumDinValue; // Size of the reserved string value
    char *pDinValue;       // Used if MM_MAX_STRING_FIELD_VALUE is not enough
	double dValue;     // For double and 32 bit integer numeric values and 
    __int64 iValue;    // For 64 bit integer values. 
	//MM_BOOLEAN kbValue;	// For binary values.
};

struct MiraMonRecord
{
    MM_EXT_DBF_N_FIELDS nMaxField;	// Number of reserved fields
	MM_EXT_DBF_N_FIELDS nNumField;	// Number of fields
	struct MiraMonFieldValue *pField;	// Value of the fields.
};

struct MiraMonDataBaseField
{
    char pszFieldName[MM_MAX_LON_FIELD_NAME_DBF+1];
    char pszFieldDescription[MM_MAX_BYTES_FIELD_DESC+1];
    enum FieldType eFieldType; // See enum FieldType
    unsigned __int32 nFieldSize; // MM_MAX_BYTES_IN_A_FIELD_EXT as maximum
    unsigned __int32 nNumberOfDecimals; // MM_MAX_BYTES_IN_A_FIELD_EXT as maximum
    MM_BOOLEAN bIs64BitInteger; // For 64 bits integer fields
};

struct MiraMonDataBase
{
    MM_EXT_DBF_N_FIELDS nNFields;
    struct MiraMonDataBaseField *pFields;
};

struct MMAdmDatabase
{
    // MiraMon Database (extended DBF)
    // Name of the extended DBF file
    char *pszExtDBFLayerName; 
     // Pointer to the extended DBF file
    FILE_TYPE *pFExtDBF;
    // Pointer to a MiraMon database (auxiliar)
    struct MM_BASE_DADES_XP *pMMBDXP;
    // How to write all it to disk
    struct MM_FLUSH_INFO FlushRecList;
    char *pRecList; // Records list  // (II mode)

    // Temporary space where to mount the DBF record.
    // Reused every time a feature is created
    char *szRecordOnCourse;
};


// MIRAMON GEOMETRY

// Top Header section
struct MM_TH 
{
    char aLayerVersion[2];
    char aLayerSubVersion;
    
    char aFileType[3]; // (PNT, ARC, NOD, POL)

    int bIs3d;
    int bIsMultipolygon; // Only apply to polygons

    unsigned char Flag;  // 1 byte: defined at DefTopMM.H
    struct MMBoundingBox hBB;
    MM_INTERNAL_FID nElemCount;  // 4/8 bytes depending on the version
    // 8/4 reserved bytes depending on the version
};


// Z Header (32 bytes)
struct MM_ZH 
{
    __int32 nMyDiskSize;
    // 16 bytes reserved
    double dfBBminz; // 8 bytes Minimum Z
    double dfBBmaxz; // 8 bytes Maximum Z
};

// Z Description
struct MM_ZD 
{
    double dfBBminz; // 8 bytes Minimum Z
    double dfBBmaxz; // 8 bytes Maximum Z
    unsigned __int32 nZCount;  // 4 bytes
    // 4 bytes reserved (Only in version 2.0)
    MM_FILE_OFFSET nOffsetZ; // 4 or 8 bytes depending on the version
};
   
struct MM_ZSection
{
    // Offset where the section begins in disk. It's a precalculated value
    // using nElemCount from LayerInfo. TH+n*CL
    MM_FILE_OFFSET ZSectionOffset;
    struct MM_ZH ZHeader;   // (I mode)
    
    // Number of pZDescription allocated
    // nMaxZDescription = nElemCount from LayerInfo
    MM_FILE_OFFSET ZDOffset;
    __int32 nZDDiskSize;
    unsigned __int64 nMaxZDescription; 
    struct MM_ZD *pZDescription; //(I mode)

    struct MM_FLUSH_INFO FlushZL;
    char *pZL;  // (II mode)
};

// Header of Arcs
struct MM_AH
{
    struct MMBoundingBox dfBB;
    MM_TIPUS_N_VERTEXS nElemCount; // 4/8 bytes depending on the version
    MM_FILE_OFFSET nOffset; // 4/8 bytes depending on the version
    MM_INTERNAL_FID nFirstIdNode; // 4/8 bytes depending on the version
    MM_INTERNAL_FID nLastIdNode; // 4/8 bytes depending on the version
    double dfLenght;
};

// Header of Nodes
struct MM_NH
{
    short int nArcsCount;
    char cNodeType;
    // 1 reserved byte
    MM_FILE_OFFSET nOffset; // 4/8 bytes depending on the version
};

// Header of Polygons
struct MM_PH
{
    // Common Arc & Polyons section
    struct MMBoundingBox dfBB;
    unsigned __int64 nArcsCount; // 4/8 bytes depending on the version
    unsigned __int64 nExternalRingsCount; // 4/8 bytes depending on the version
    unsigned __int64 nRingsCount; // 4/8 bytes depending on the version
    MM_FILE_OFFSET nOffset; // 4/8 bytes depending on the version
    double dfPerimeter;
    double dfArea;
    //struct GEOMETRIC_I_TOPOLOGIC_POL GeoTopoPol;
};

/*  Every MiraMon file is composed as is specified in documentation.
    Here are the structures to every file where we can find two ways
    of keeping the information in memory (to be, finally, flushed to the disk)
        * (I mode)Pointers to structs that keep information that changes every time
          a feature is added. They will be written at the end on disk.
        * (II mode)Memory blocs that are used as buffer blocs to store information that
          is going to be flushed (as are) at the disc periodically instead 
          of writing them to the disc every time a Feature is added (not 
          eficient). The place where they are going to be flushed depends
          on one variable: the number of elements of the layer.
*/

// MiraMon Point Layer: TH, List of CL (coordiantes), ZH, ZD, ZL
struct MiraMonPointLayer
{
    // Name of the layer with extension
    char *pszLayerName; 
    FILE_TYPE *pF;
    
    // Coordinates x,y of the points
    struct MM_FLUSH_INFO FlushTL;
    char *pTL; // (II mode)
    char *pszTLName; // Temporary file where to flush
    FILE_TYPE *pFTL; // Pointer to temporary file where to flush
    
    // Z section
    // Temporal file where the Z coordinates are stored
    // if necessary
    char *psz3DLayerName;
    FILE_TYPE *pF3d; 
    struct MM_ZSection pZSection;

    // MiraMon Database (extended DBF)
    struct MMAdmDatabase MMAdmDB;
};

struct MiraMonNodeLayer
{
    char *pszLayerName; // Name of the layer with extension
    FILE_TYPE *pF;
    
    // Header of every node
    __int32 nSizeNodeHeader;
    unsigned __int64 nMaxNodeHeader; // Number of pNodeHeader allocated
    struct MM_NH *pNodeHeader;// (I mode)
    
    // NL: arcs confuent to node 
    struct MM_FLUSH_INFO FlushNL; // (II mode)
    char *pNL; // 
    char *pszNLName; // Temporary file where to flush
    FILE_TYPE *pFNL; // Pointer to temporary file where to flush

    struct MMAdmDatabase MMAdmDB;
};

struct MiraMonArcLayer
{
    char *pszLayerName; // Name of the layer with extension
    FILE_TYPE *pF;

    // Temporal file where the Z coordinates are stored
    // if necessary
    char *psz3DLayerName;
    FILE_TYPE *pF3d; 
                
    // Header of every arc
    __int32 nSizeArcHeader;
    unsigned __int64 nMaxArcHeader; // Number of allocated pArcHeader 
    struct MM_AH *pArcHeader;// (I mode)

    // AL Section
    struct MM_FLUSH_INFO FlushAL;
    int nALElementSize; //    16 // Two double coordinates
    char *pAL; // Arc List  // (II mode)
    char *pszALName; // Temporary file where to flush
    FILE_TYPE *pFAL; // Pointer to temporary file where to flush
        
    // Z section
    struct MM_ZSection pZSection;

    // Node layer associated to the arc layer
    struct MM_TH TopNodeHeader;
    struct MiraMonNodeLayer MMNode;

    // An Arc layer can store some ellipsoidal information
    double *pEllipLong;
    struct SNY_TRANSFORMADOR_GEODESIA *GeodesiaTransform;

    // Private data
    unsigned __int64 nMaxArcVrt; // Number of allocated 
    struct ARC_VRT_STRUCTURE *pArcVrt;
    MM_FILE_OFFSET nOffsetArc; // It's an auxiliary offset

    struct MMAdmDatabase MMAdmDB;
};

struct MiraMonPolygonLayer
{
    char *pszLayerName; // Name of the layer with extension
    FILE_TYPE *pF;

    // PS part
    struct MM_FLUSH_INFO FlushPS;
    int nPSElementSize; 
    char *pPS;  // Polygon side (II mode)
    char *pszPSName; // Temporary file where to flush
    FILE_TYPE *pFPS; // Pointer to temporary file where to flush
    
    // Header of every polygon
    unsigned __int64 nMaxPolHeader; // Number of pPolHeader allocated
    int nPHElementSize;
    struct MM_PH *pPolHeader;// (I mode)
    
    // PAL
    struct MM_FLUSH_INFO FlushPAL;
    char *pPAL; // Polygon Arc List  // (II mode)
    char *pszPALName; // Temporary file where to flush
    FILE_TYPE *pFPAL; // Pointer to temporary file where to flush

    // Arc layer associated to the arc layer
    struct MM_TH TopArcHeader;
    struct MiraMonArcLayer MMArc;

    struct MMAdmDatabase MMAdmDB;
};

#define MM_VECTOR_LAYER_LAST_VERSION    1
#define CheckMMVectorLayerVersion(a,r){if((a)->Version!=MM_VECTOR_LAYER_LAST_VERSION)return (r);}

// Information that allows to reuse memory stuff when
// features are being read
struct MiraMonFeature
{
    // Number of parts
    unsigned __int64 nNRings; // =1 for lines and points
    unsigned __int64 nIRing; // The ring is being processed
    
    // Number of reserved elements in *pNCoord
    MM_TIPUS_N_VERTEXS nMaxpNCoord; 
    MM_TIPUS_N_VERTEXS *pNCoord; // [0]=1 for lines and points

    // Number of reserved elements in *pCoord
    MM_TIPUS_N_VERTEXS nMaxpCoord; 
    // Coordinate index thats is being processed
    MM_TIPUS_N_VERTEXS nICoord; 
    // List of the coordinates of the feature
    struct MM_POINT_2D *pCoord; 

    // Number of reserved elements in *pbArcInfo
    unsigned __int64 nMaxpbArcInfo;
	int *pbArcInfo; // In case of multipolygons, for each ring:
						 // TRUE if it's a outer ring,
						 // FALSE if it's a inner ring.

    // List of the Z-coordinates (as many as pCoord)
    // Number of reserved elements in *pZCoord
    unsigned __int64 nMaxpZCoord; 
    double *pZCoord; 

    // Records of the feature
	MM_NUMERATOR_RECORD nNumRecords;
    // Number of reserved elements in *pRecords
    MM_NUMERATOR_RECORD nMaxRecords; 
	struct MiraMonRecord *pRecords;
};

// MIRAMON OBJECT: Contains everything
struct MiraMonLayerInfo
{
    // Version of the structure
    __int32 Version; 

    // Version of the layer
    // MM_32BITS_LAYER_VERSION: less than 2 Gb files
    // MM_64BITS_LAYER_VERSION: more than 2 Gb files
    char LayerVersion;

    char *pszSrcLayerName;
    
    char pszFlags[10]; // To Open the file
    int bIsPolygon;
    int bIsArc; // Also 1 in a polygon layer
    int bIsPoint;
    
    // Final number of elements of the layer. 
    MM_INTERNAL_FID nFinalElemCount; // Real element count after conversion
    
    // Header of the layer
    __int32 nHeaderDiskSize;
    struct MM_TH TopHeader;

    int eLT;    // Type of layer: Point, line or polygon (3d or not)
    int bIsBeenInit; // 1 if layer has already been initialized
    int bNameNeedsCorrection; // 1 if name needs the extension to be added

    // Point layer
    struct MiraMonPointLayer MMPoint;

    // Arc layer
    struct MiraMonArcLayer MMArc;
    
    // Polygon layer 
    struct MiraMonPolygonLayer MMPolygon;

    // Offset used to write features.
    MM_FILE_OFFSET OffsetCheck;

    // EPSG code of the coordinate system information.
    char *pSRS; // User has to free it.

    // Layer database read from origin.
    struct MiraMonDataBase *pLayerDB;

    // Charset of DBF files (same for all)
    //  MM_JOC_CARAC_UTF8_DBF
    //  MM_JOC_CARAC_ANSI_DBASE;
    MM_BYTE nCharSet;
    
    // This is used only to write temporary stuff
    char szNFieldAux[MM_MAX_AMPLADA_CAMP_N_DBF];
    // Dinamic string that is used as temporary buffer
    // with variable size as needed. Its value is 
    // very temporary. Copy in a safe place to save its value.
    MM_NUMERATOR_DBF_FIELD_TYPE nNumStringToOperate;
    char *szStringToOperate;
};

enum DataType {MMDTByte, MMDTInteger, MMDTuInteger, 
               MMDTLong, MMDTReal, MMDTDouble, MMDT4bits};
enum TreatmentVariable {MMTVQuantitativeContinuous, MMTVOrdinal, MMTVCategorical};

#ifdef GDAL_COMPILATION
CPL_C_END // Necessary for compiling in GDAL project
#endif
#endif //__MM_GDAL_DRIVER_STRUCTS_H
