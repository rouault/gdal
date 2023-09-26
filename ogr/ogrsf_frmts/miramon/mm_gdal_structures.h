#ifndef __MM_GDAL_STRUCTURES_H
#define __MM_GDAL_STRUCTURES_H
/* -------------------------------------------------------------------- */
/*      Constants used in GDAL and in MiraMon                           */
/* -------------------------------------------------------------------- */
#ifdef GDAL_COMPILATION
#include "cpl_conv.h"   // For FILE_TYPE
#include "mm_gdal_constants.h"
#else
#include "F64_str.h"	// For FILE_64
#include "mm_gdal\mm_gdal_constants.h"
#endif



#ifdef GDAL_COMPILATION
CPL_C_START // Necessary for compiling in GDAL project
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

struct MM_CAMP // FIELD
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

    				#define MM_OFFSET_BYTESxCAMP_CAMP_CLASSIC  16 
    				#define MM_OFFSET_BYTESxCAMP_CAMP_ESPECIAL 21 

    // Number of bytes of a field
	MM_TIPUS_BYTES_PER_CAMP_DBF  BytesPerCamp; 

    // Acumulated bytes before a field
	MM_TIPUS_BYTES_ACUMULATS_DBF BytesAcumulats; 

    // Not used in GDAL
	char *(separador[MM_NUM_IDIOMES_MD_MULTIDIOMA]); 

    // Description of the field
	char DescripcioCamp[MM_NUM_IDIOMES_MD_MULTIDIOMA][MM_MAX_LON_DESCRIPCIO_CAMP_DBF];

    // Show field
	MM_BYTE mostrar_camp;

    // Not used in GDAL: Simbolize field
	MM_BYTE simbolitzable;

    // Not used in GDAL: Field descriptions as a link
	MM_NUMERATOR_DBF_FIELD_TYPE CampDescHipervincle; 

    // Not used in GDAL: Content as a link
    MM_BOOLEAN EsHipervincle;

    // Not used in GDAL: Inserted content
    MM_BOOLEAN EsContingutIncrustat;

    // Not used in GDAL: Content as expression
    MM_BOOLEAN EsExpressio; 

	MM_BYTE ProveDeNivJerarqSup; /* En teoria pot prendre un dels seg�ents valors:
				    	PROVE_DE_NIV_JERARQ_SUP_NO, PROVE_DE_NIV_JERARQ_SUP_FULL,
				        PROVE_DE_NIV_JERARQ_SUP_SERIE o PROVE_DE_NIV_JERARQ_SUP_MULTISERIE,
                        per� pel disseny d'her�ncia realitzat nom�s hauria a de ser
                        pot ser PROVE_DE_NIV_JERARQ_SUP_NO o PROVE_DE_NIV_JERARQ_SUP_SERIE.*/

    MM_TIPUS_NUMERADOR_TAULA_ASSOC NTaulesAssoc;
    struct BDXP_O_BDODBC * TaulaAssoc; /* Usat en estructures de bases de
    							dades de RELs de la versi� 4. El seu contingut
                                nom�s t� sentit quan
                                	mostrar_camp==CAMP_QUE_MOSTRA_TAULA_BDXP_O_BDODBC.
                                En cas contrari queda indeterminat.*/
    MM_BYTE TractamentVariable; /* Indica si el camp cont� una variable de tipus
    							"quantitatiu continu", "ordinal" ("qualitatiu")
                                o "categ�ric". Vegeu els define CAMP_INDETERMINAT,
                                CAMP_CATEGORIC, CAMP_ORDINAL i
                                CAMP_QUANTITATIU_CONTINU a DefBD.h i les
                                funcions TipusCampDBF_a_TractamentVariable() i
                                DonaTractamentVariable_o_Defecte().  */
    char unitats[MM_MAX_LON_UNITATS_CAMP];
    MM_BOOLEAN MostrarUnitats;

    //char NODATADef[MAX_NODATA_DEF]; 

    MM_BYTE AmpleDesitjat; 
    MM_BYTE AmpleDesitjatOriginal; 

#define MM_MAX_LON_RESERVAT_1_CAMP_BD_XP 4
	MM_BYTE reservat_1[MM_MAX_LON_RESERVAT_1_CAMP_BD_XP];

#define MM_OFFSET_RESERVAT2_BYTESxCAMP_CAMP_ESPECIAL 3
#define MM_OFFSET_RESERVAT2_OFFSET_NOM_ESTES	 7
#define MM_OFFSET_RESERVAT2_MIDA_NOM_ESTES		11
#define MM_MAX_LON_RESERVAT_2_CAMP_BD_XP 13
	MM_BYTE reservat_2[MM_MAX_LON_RESERVAT_2_CAMP_BD_XP];
	MM_BYTE MDX_camp_flag;
 	MM_BYTE TipusCampGeoTopo;
};

struct MM_BASE_DADES_XP // MiraMon Database Structure
{
    // Extended DBF file name
	char szNomFitxer[MM_MAX_PATH];
    
    // Temporal table
	MM_BOOLEAN EsTaulaTemporal;

    // Charset of the DBF
	//struct CARACT_FITX caract_fitxerDBF;

    FILE_TYPE  *pfBaseDades;
	//FILE_MEM *pfBaseDades_mem;

    // Charset of the DBF
	MM_BYTE JocCaracters; 
	//BYTE InfoJocCaracExterna;
	char ModeLectura[4];
	MM_NUMERATOR_RECORD nfitxes;
	MM_TIPUS_BYTES_ACUMULATS_DBF BytesPerFitxa;
	MM_NUMERATOR_DBF_FIELD_TYPE ncamps;
	struct MM_CAMP huge * Camp;
    MM_TIPUS_OFFSET_PRIMERA_FITXA OffsetPrimeraFitxa;
	MM_NUMERATOR_DBF_FIELD_TYPE CampIdGrafic;
    MM_NUMERATOR_DBF_FIELD_TYPE CampIdEntitat;
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

    //TIPUS_NUMERADOR_CAMP_DBF CampQueTeFitxerObert;
    //TIPUS_NUMERADOR_CAMP_DBF CampRelacional;
    //void huge * IndexICampRelacional;
    //MM_BOOLEAN PucFerCercaBinaria;

    #define MM_ES_DBF_ESTESA(versio_dbf) (((versio_dbf)==MM_MARCA_VERSIO_1_DBF_ESTESA)?TRUE:FALSE)
};
#ifdef GDAL_COMPILATION
CPL_C_END // Necessary for compiling in GDAL project
#endif
#endif //__MM_GDAL_STRUCTURES_H
