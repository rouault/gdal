.. _rfc-100:

===================================================================
RFC 100: Custom GDAL transformers
===================================================================

============== =============================================
Author:        Even Rouault
Contact:       even.rouault @ spatialys.com
Author:        Frank Warmerdam
Contact:       frank @ planet.com
Started:       2024-May-07
Status:        Draft
Target:        GDAL 3.10
============== =============================================

Summary
-------

This RFC introduces a new public function to be able to register custom spatial
point transformers for use by gdalwarp and warped VRT XML.

Motivation
----------

Let's start by recalling that one key component of the GDAL image warping logic relies
on the ability to transform from source image pixel/line coordinates to target
image pixel/line coordinates, and the reverse. The creation of such transformer,
of type :cpp:type:`GDALTransformerFunc`, is done by :cpp:func:`GDALCreateGenImgProjTransformer2`.

This transformation chains 3 transformation steps (given here for the source to target path):

- transform from source image pixel/line coordinates to source georeferenced coordinates.
  This transformation can just use a geotransform matrix in the most simple case,
  a GCP polynomial or TPS-based transformers, a RPC transformation or a Geolocation array
  transformation.

- transform from source georeferenced coordinates to target georeferenced coordinates.
  This is accomplished using a :cpp:class:`OGRCoordinateTransformation`.

- transform from target georeferenced coordinates to target image pixel/line coordinates.
  This is very similar to the first step. In most cases, this will use the inverse geotransform
  matrix of the target dataset, but this can also be any o the GCP, TPS, RPC or geolocation
  array transformers.

Currently, there is no way to plug a custom external transformer in the first and
third steps of :cpp:func:`GDALCreateGenImgProjTransformer2`. This RFC aims at
providing this capability for 2 reasons:

- currently the logic to instantiate the GCP, TPS, RPC and Geolocation arrays is
  hardcoded directly in GDALCreateGenImgProjTransformer2(), and it is done twice:
  once for the first step and once for the third step. It would be cleaner to have
  GDALCreateGenImgProjTransformer2() query for a list of registered transformers,
  and let them instantiate themselves.

- and allow GDAL users that have developed custom transformers, typically remote
  sensing operators that have developed camera models for their own sensor, to be
  able to write GDAL plugins where they can register their transformer and use
  the GDAL warping machinery with it. Such custom transformers can potentially
  use the `Community Sensor Model <https://gwg.nga.mil/gwg/focus-groups/Community_Sensor_Model_Working_Group_(CSMWG).html>`__
  formalism, and have a thin adaptation layer too expose it as a
  :cpp:type:`GDALTransformerFunc`.

This RFC also gives a way for a custom transformer to register a callback
function to instantiate a transformer from a XML serialization of a warped VRT.
In the below example, this is the ability to instantiate a GCP transformer from
the content of the ``GCPTransformer`` element.

.. code-block:: xml

    <VRTDataset rasterXSize="20" rasterYSize="20" subClass="VRTWarpedDataset">
      <!-- snip -->
              <GenImgProjTransformer>
                <SrcGCPTransformer>
                  <GCPTransformer>  <!-- This RFC adds the ability to have here a custom XML node -->
                    <Order>1</Order>
                    <Reversed>0</Reversed>
                    <Refine>1</Refine>
                    <Tolerance>1</Tolerance>
                    <GCPList>
                      <GCP Id="" Pixel="0" Line="0" X="0" Y="20" />
                      <GCP Id="" Pixel="0" Line="0" X="0" Y="20" />
                      <GCP Id="" Pixel="0" Line="20" X="0" Y="0" />
                      <GCP Id="" Pixel="0" Line="20" X="0" Y="0" />
                      <GCP Id="" Pixel="20" Line="20" X="2000" Y="0" />
                      <GCP Id="" Pixel="20" Line="0" X="20" Y="20" />
                      <GCP Id="" Pixel="20" Line="0" X="20" Y="20" />
                      <GCP Id="" Pixel="20" Line="40" X="4000" Y="0" />
                      <GCP Id="" Pixel="20" Line="20" X="20" Y="0" />
                      <GCP Id="" Pixel="20" Line="20" X="20" Y="0" />
                    </GCPList>
                  </GCPTransformer>
                </SrcGCPTransformer>
                <DstGeoTransform>0,1,0,20,0,-1</DstGeoTransform>
                <DstInvGeoTransform>0,1,0,20,0,-1</DstInvGeoTransform>
              </GenImgProjTransformer>
        <!-- snip -->
    </VRTDataset>


Details
-------

API additions
+++++++++++++

In :file:`gdal_alg.h`, a new function GDALRegisterTransformer() is added.

.. code-block:: c

    /** Registers a new spatial point transformer.
     *
     * The transformer instance returned by pfnDeserializeFunc() or
     * pfnTransformerCreateForGenImgTransformerFunc() should be a structure whose
     * first member is an instance of GDALTransformerInfo.
     *
     * @param pszTransformName Transformer name. Must not be NULL. It must be
     *                         unique as used as a key (case insensitive)
     *                         The following keys are reserved as GDAL builtins:
     *                         "GenImgProjTransformer", "ReprojectionTransformer",
     *                         "GCPTransformer", "TPSTransformer", "GeoLocTransformer",
     *                         "RPCTransformer", "ApproxTransformer".
     *                         This is used by GDALDeserializeTransformer() to
     *                         determine which transformer is appropriate to
     *                         instantiate a transformer from its XML serialization,
     *                         and must thus be a valid XML element name (possibly
     *                         namespaced)
     * @param pfnDeserializeFunc XML deserizaliation function. For example used to
     *                           instantiate a transformer instance from a warped VRT.
     * @param pfnTransformerCreateForGenImgTransformerFunc Function to instantiate
     * a transformer for use by GDALGenImgProjTransform() (typically for gdalwarp).
     * If NULL, the transformer cannot be use by the GDALCreateGenImgProjTransformer()
     * family of functions.
     * @since GDAL 3.10
     * @return an opaque pointer that can be passed to GDALUnregisterTransformer(),
     * or NULL in case of error.
     */
    void CPL_DLL *
    GDALRegisterTransformer(const char *pszTransformName,
                            GDALTransformDeserializeFunc pfnDeserializeFunc,
                            GDALTransformerCreateForGenImgTransformer
                                pfnTransformerCreateForGenImgTransformerFunc);


The details for introduced function pointer type GDALTransformerCreateForGenImgTransformer are:

.. code-block:: c

    /** Function pointer that instantiates a transformer instance for use by the
     * GDALCreateGenImgProjTransformer() family of functions.
     *
     * The transformer instance should be a structure whose first member is an
     * instance of GDALTransformerInfo.
     *
     * @param pszMethod Transformation method. May be NULL, in which case the
     *                  callback is free to decide, from the content of the dataset
     *                  and transformer options, if it can be instantiated.
     * @param bIsSrcTransformer true if the transformer is used as the source
     *                          transformer of GDALGenImgProjTransform.
     *                          false if the transformer is used as the target
     *                          transformer of GDALGenImgProjTransform.
     * @param hDS source or target dataset, depending of bIsSrcTransformer.
     *            The passed value is not NULL.
     * @param papszTransformOptions Transformer options. May be NULL.
     * @param[in,out] phSRS Pointer to a OGRSpatialReferenceH. The input *phSRS may be
     *                   set. The callback may assign a new value to *phSRS (to be
     *                   released with OSRRelease(). If the callback needs to free
     *                   the passed *phSRS value, it must do so with OSRRelease().
     * @return a new transformer instance, or NULL in case of error or if
     * pszMethod == NULL and the callback does not recognize it should be
     * instantiated.
     */
    typedef void *(*GDALTransformerCreateForGenImgTransformer)(
        const char *pszMethod, bool bIsSrcTransformer, GDALDatasetH hDS,
        char **papszTransformOptions, OGRSpatialReferenceH *phSRS);


GDALTransformDeserializeFunc already existed as a private/undocument typedef,
and is now exposed in the API (ith a change from ``CPLXMLNode*`` to ``const CPLXMLNode*``)

.. code-block:: c

    /** Function pointer that takes a XML tree as argument and returns a
     * transformer instance.
     *
     * The transformer instance returned should be a structure whose first member
     * is an instance of GDALTransformerInfo.
     *
     * @param psTree XML tree
     * @return a new transformer instance, or NULL in case of error
     */
    typedef void *(*GDALTransformDeserializeFunc)(const CPLXMLNode *psTree);


The 2 callbacks pfnDeserializeFunc and pfnTransformerCreateForGenImgTransformerFunc
should return a ``void*`` pointer that can be cast as a ``GDALTransformerInfo*``.

.. code-block:: c

    /** Signature to be set in GDALTransformerInfo::abySignature member */
    #define GDAL_GTI2_SIGNATURE "GTI2"

    /** Structure that must be instantiated as the first member of a transformer
     * instance.
     */
    typedef struct
    {
        /** Signature. Should be filled with GDAL_GTI2_SIGNATURE. */
        GByte abySignature[4];

        /** Class name. Should uniquely identify the transformer. */
        const char *pszClassName;

        /** Transformer callback. */
        GDALTransformerFunc pfnTransform;

        /** Cleanup callback. */
        void (*pfnCleanup)(void *pTransformerArg);

        /** XML serialization callback. */
        CPLXMLNode *(*pfnSerialize)(void *pTransformerArg);

        /** Callback to instantiate a transformer instance similar to the passed one. */
        void *(*pfnCreateSimilar)(void *pTransformerArg, double dfSrcRatioX,
                                  double dfSrcRatioY);
    } GDALTransformerInfo;


This is typically done by setting a GDALTransformerInfo field as the first
member of the instance of the structure returned by the callback. For example,
for the builtin GCP transformer:

.. code-block:: c

    struct GCPTransformInfo
    {
        GDALTransformerInfo sTI{};
        ... other fields ...
    };


This can be seen as an emulation of implementing a C++ interface in C. Note that
this mechanism is not new to this RFC and already pre-exists in GDAL for its
existing builtin transformers.


For completeness, a unregistration function is provided, although there is no
obligation to call it (:cpp:func:`GDALDestroy` will automatically unregister
all registered functions)

.. code-block:: c

    /** Unregisters a spatial point transformer previously registered with
     * GDALRegisterTransformer().
     *
     * @param pData Value returned by GDALRegisterTransformer()
     * @since GDAL 3.10
     */
    void CPL_DLL GDALUnregisterTransformer(void *pData);


API removals
++++++++++++

GDALRegisterTransformDeserializer() and GDALUnregisterTransformDeserializer()
currently present in :file:`gdal_alg_priv.h` are removed, and replaced by
GDALRegisterTransformer() and GDALUnregisterTransformer(). See below section
about Backward compatibility for more details.

Code changes
++++++++++++

An internal GDALRegisterBuiltinTransformersUnderLock() function is added to
register all built-in transformers (GCP polynomial, TPS, RPC, Geolocation array),
using the new GDALRegisterTransformer() API. The instantiation of such
transformer that used to be hard-coded in :cpp:func:`GDALCreateGenImgProjTransformer2`
is now moved into dedicated functions whose signature match the
GDALTransformerCreateForGenImgTransformer callback function type.
GDALCreateGenImgProjTransformer2() is modified to iterate over the registered
functions, in the order they are registered, to find a transformer that match the
requested transformation method.

There are 2 possibilities:

- either GDALCreateGenImgProjTransformer2 is called with an explicit METHOD/SRC_METHOD/DST_METHOD
  transformer option, and then each GDALTransformerCreateForGenImgTransformer
  probed callback can know for sure, if it should instantiate itself or not from
  the value of pszMethod.

- or pszMethod==NULL is passed to the callback, in which case, it must use other
  contextual elements in the passed dataset (presence of specific metadata, GCPs, etc.)
  or other transformation options to infer if it should instantiate itself.
  Note that a custom transformer, registered from a GDAL plugin loaded by the GDAL
  plugin machinery, will be registered before the builtin functions, and thus will
  be probed before them.

Those 2 possibilities are currently used. The first one for example if using
the ``-order``, ``-tps``, ``-rpc`` or ``-geoloc`` swiches of gdalwarp. And the
second one if not using them.

GDALDeserializeTransformer() is also modified to remove the hard-coded tests
to instantiate the appropriate transformer from the name of the XML node, and
rather use the map of transformers built by GDALRegisterTransformer().

SWIG bindings
-------------

No changes

Testing
-------

Existing tests are sufficient to check most of the new capabilities, given that existing
transformers are registered using the new mechanism.

A new test is added in the C++ autotest suite to check GDALUnregisterTransformer().

Backward compatibility
----------------------

An earlier related effort had been done in 2010 (https://github.com/OSGeo/gdal/commit/2a17af8e55cc7836fd009da9e8249142fdfe85e1)
where a GDALRegisterTransformDeserializer() function had been introduced for the
internal needs of the overview logic in warped VRT. Later in 2014, the use of
that function was dropped per https://github.com/OSGeo/gdal/commit/da640477a69fa56596253ed07741c8356717765a .
It has no longer been used internally by GDAL since then. Hence this function
is removed in the candidate implementation and replaced by the new proposed
GDALRegisterTransformer().
For completeness, one should note that this past GDALRegisterTransformDeserializer()
function was exported per ticket https://trac.osgeo.org/gdal/ticket/5392 for the
need of one user. Hence this RFC will perhaps impact a few external users.
The impact on the overhaul community should be modest though as this function
was in the gdal_alg_priv.h header and not documented. It is for example not used
by any software referenced in the Debian source explorer. The migration path to
using GDALRegisterTransformer() should be simple enough, as it just takes an
extra argument that can be set to NULL (for equivalent functionality, that is
the ability to read a serialized warped VRT with a custom transformer).

Related issues and PRs
----------------------

- Candidate implementation: https://github.com/rouault/gdal/tree/custom_transformer

Voting history
--------------

TBD
