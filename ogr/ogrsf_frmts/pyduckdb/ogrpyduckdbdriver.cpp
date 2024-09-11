#include "cpl_multiproc.h"
#include "cpl_spawn.h"
#include "ogrsf_frmts.h"
#include "ogr_recordbatch.h"
#include "gdalpython.h"

#include <algorithm>
#include <utility>

using namespace GDALPy;

constexpr const char *DRIVER_NAME = "PyDuckDB";

extern "C" CPL_DLL void RegisterOGRPyDuckDB();

class OGRPyDuckDBDataset final : public GDALDataset
{
  private:
    friend class OGRPyDuckDBLayer;

    std::string m_osFilename{};
    std::string m_osConnectionName{};
    std::string m_osSQL{};
    PyObject *m_poOGRPyGetArrowStream = nullptr;
    PyObject *m_poOGRPyConnect = nullptr;
    PyObject *m_poConnection = nullptr;
    PyObject *m_poCapsule = nullptr;
    ArrowArrayStream *m_psArrowArrayStream = nullptr;
    std::unique_ptr<OGRLayer> m_poLayer{};

    std::unique_ptr<GDALDataset> CreateMemLayer();

    bool Connect();
    bool InitializeDuckDB();

    CPL_DISALLOW_COPY_ASSIGN(OGRPyDuckDBDataset)

  public:
    OGRPyDuckDBDataset() = default;
    ~OGRPyDuckDBDataset() override;

    static GDALDataset *Open(GDALOpenInfo *poOpenInfo);

    bool GetArrowArrayStream();
    void ResetArrowArrayStream();

    int GetLayerCount() override
    {
        return m_poLayer ? 1 : 0;
    }

    OGRLayer *GetLayer(int nIdx) override
    {
        return nIdx == 0 ? m_poLayer.get() : nullptr;
    }

    static std::unique_ptr<OGRPyDuckDBDataset>
    Create(const std::string &osConnectionName, OGRPyDuckDBDataset *poMainDS,
           const std::string &osFilename, const std::string &osSQL);
};

class OGRPyDuckDBLayer final : public OGRLayer
{
    friend class OGRPyDuckDBDataset;

    OGRPyDuckDBDataset *m_poDS = nullptr;
    std::unique_ptr<GDALDataset> m_poMemDS{};
    OGRLayer *m_poMemLayer = nullptr;
    OGRFeatureDefn *m_poLayerDefn = nullptr;
    GIntBig m_nNextFID = 0;
    int m_nPage = 0;
    bool m_bEndOfGetNext = false;

    struct ArrowStreamPrivate
    {
        struct ArrowArrayStream *stream_ori = nullptr;
        bool bGetNextCalled = false;
        bool bReleased = true;

        static const char *GetLastError(struct ArrowArrayStream *self)
        {
            auto psPrivate =
                static_cast<ArrowStreamPrivate *>(self->private_data);
            return psPrivate->stream_ori->get_last_error(psPrivate->stream_ori);
        }

        static int GetNext(struct ArrowArrayStream *self,
                           struct ArrowArray *array)
        {
            auto psPrivate =
                static_cast<ArrowStreamPrivate *>(self->private_data);
            psPrivate->bGetNextCalled = true;
            memset(array, 0, sizeof(*array));
            CPLDebug("OGRPyDuckDB", "get_next(): start");
            const int ret =
                psPrivate->stream_ori->get_next(psPrivate->stream_ori, array);
            CPLDebug("OGRPyDuckDB", "get_next(): end: ret = %d, length = %d",
                     ret, int(array->length));
            return ret;
        }

        static int GetSchema(struct ArrowArrayStream *self,
                             struct ArrowSchema *schema)
        {
            auto psPrivate =
                static_cast<ArrowStreamPrivate *>(self->private_data);
            memset(schema, 0, sizeof(*schema));
            return psPrivate->stream_ori->get_schema(psPrivate->stream_ori,
                                                     schema);
        }

        static void Release(struct ArrowArrayStream *self)
        {
            auto psPrivate =
                static_cast<ArrowStreamPrivate *>(self->private_data);
            psPrivate->stream_ori = nullptr;
            psPrivate->bReleased = true;
            self->release = nullptr;
        }
    };

    ArrowStreamPrivate m_oArrowStreamPrivate{};

    CPL_DISALLOW_COPY_ASSIGN(OGRPyDuckDBLayer)

  public:
    explicit OGRPyDuckDBLayer(OGRPyDuckDBDataset *poDS)
        : m_poDS(poDS), m_poMemDS(poDS->CreateMemLayer())
    {
        if (m_poMemDS)
        {
            m_poMemLayer = m_poMemDS->GetLayer(0);
            m_poLayerDefn = m_poMemLayer->GetLayerDefn()->Clone();
            SetDescription(m_poLayerDefn->GetName());
            m_poLayerDefn->Reference();
        }
    }

    ~OGRPyDuckDBLayer() override
    {
        if (m_poLayerDefn)
            m_poLayerDefn->Release();
    }

    OGRFeatureDefn *GetLayerDefn() override
    {
        return m_poLayerDefn;
    }

    void ResetReading() override
    {
        m_nNextFID = 0;
        if (m_nPage > 1)
        {
            m_bEndOfGetNext = false;
            m_nPage = 0;
            m_poMemLayer = nullptr;
            m_poMemDS.reset();
            m_poDS->ResetArrowArrayStream();
        }
        else if (m_poMemLayer)
        {
            m_poMemLayer->ResetReading();
        }
    }

    OGRFeature *GetNextFeature()
    {
        while (true)
        {
            auto poSrcFeat = std::unique_ptr<OGRFeature>(
                m_poMemLayer ? m_poMemLayer->GetNextFeature() : nullptr);
            if (poSrcFeat)
            {
                auto poFeat = std::make_unique<OGRFeature>(m_poLayerDefn);
                poFeat->SetFrom(poSrcFeat.get());
                poFeat->SetFID(++m_nNextFID);

                if (m_poFilterGeom && !FilterGeometry(poFeat->GetGeometryRef()))
                    continue;
                if (m_poAttrQuery && !m_poAttrQuery->Evaluate(poFeat.get()))
                    continue;

                return poFeat.release();
            }
            else
            {
                if (m_bEndOfGetNext)
                    return nullptr;
                if (!m_poDS->m_psArrowArrayStream)
                {
                    if (!m_poDS->GetArrowArrayStream())
                        m_bEndOfGetNext = true;
                }
                struct ArrowArray sArray;
                memset(&sArray, 0, sizeof(sArray));
                CPLDebug("OGRPyDuckDB", "get_next(): start");
                const int nRet = m_poDS->m_psArrowArrayStream->get_next(
                    m_poDS->m_psArrowArrayStream, &sArray);
                CPLDebug("OGRPyDuckDB",
                         "get_next(): end: ret = %d, length = %d", nRet,
                         int(sArray.length));
                if (nRet != 0 || sArray.length == 0)
                {
                    m_bEndOfGetNext = true;
                    if (sArray.release)
                        sArray.release(&sArray);
                    return nullptr;
                }
                if (m_nPage > 0)
                {
                    // Re-create an empty MEM layer
                    m_poMemDS = m_poDS->CreateMemLayer();
                    if (!m_poMemDS)
                    {
                        m_bEndOfGetNext = true;
                        if (sArray.release)
                            sArray.release(&sArray);
                        m_poMemLayer = nullptr;
                        return nullptr;
                    }
                    m_poMemLayer = m_poMemDS->GetLayer(0);
                }
                m_nPage++;
                struct ArrowSchema sSchema;
                memset(&sSchema, 0, sizeof(sSchema));
                m_poDS->m_psArrowArrayStream->get_schema(
                    m_poDS->m_psArrowArrayStream, &sSchema);
                m_poMemLayer->WriteArrowBatch(&sSchema, &sArray);
                if (sSchema.release)
                    sSchema.release(&sSchema);
                if (sArray.release)
                    sArray.release(&sArray);
            }
        }
    }

    bool GetArrowStream(struct ArrowArrayStream *out_stream,
                        CSLConstList = nullptr) override
    {
        memset(out_stream, 0, sizeof(*out_stream));
        if (!m_oArrowStreamPrivate.bReleased)
        {
            CPLError(
                CE_Failure, CPLE_NotSupported,
                "Must release current ArrowStream before fetching a new one");
            return false;
        }
        if (m_oArrowStreamPrivate.bGetNextCalled)
        {
            CPLDebug("OGRPyDuckDB", "Calling GetArrowArrayStream()");
            if (!m_poDS->GetArrowArrayStream())
                return false;
        }
        m_oArrowStreamPrivate.stream_ori = m_poDS->m_psArrowArrayStream;
        m_oArrowStreamPrivate.bReleased = false;
        m_oArrowStreamPrivate.bGetNextCalled = false;
        out_stream->get_last_error = ArrowStreamPrivate::GetLastError;
        out_stream->get_schema = ArrowStreamPrivate::GetSchema;
        out_stream->get_next = ArrowStreamPrivate::GetNext;
        out_stream->private_data = &m_oArrowStreamPrivate;
        out_stream->release = ArrowStreamPrivate::Release;
        return true;
    }

    int TestCapability(const char *pszCap) override
    {
        if (EQUAL(pszCap, OLCFastGetArrowStream))
            return true;
        if (EQUAL(pszCap, OLCFastFeatureCount))
            return !m_poFilterGeom && !m_poAttrQuery &&
                   !m_poDS->m_osFilename.empty();
        return false;
    }

    GIntBig GetFeatureCount(int bForce = true) override;
};

GIntBig OGRPyDuckDBLayer::GetFeatureCount(int bForce)
{
    if (!m_poFilterGeom && !m_poAttrQuery && !m_poDS->m_osFilename.empty())
    {
        const auto osCountSQL =
            std::string("SELECT COUNT(*) FROM '")
                .append(CPLString(m_poDS->m_osFilename).replaceAll("'", "''"))
                .append("'");
        auto poFCDS = OGRPyDuckDBDataset::Create(
            m_poDS->m_osConnectionName, m_poDS, std::string(), osCountSQL);
        if (poFCDS)
        {
            if (auto poLayer = poFCDS->GetLayer(0))
            {
                auto poFeature =
                    std::unique_ptr<OGRFeature>(poLayer->GetNextFeature());
                if (poFeature)
                {
                    return poFeature->GetFieldAsInteger64(0);
                }
            }
        }
    }
    return OGRLayer::GetFeatureCount(bForce);
}

OGRPyDuckDBDataset::~OGRPyDuckDBDataset()
{
    m_poLayer.reset();
    ResetArrowArrayStream();

    if (m_poConnection)
    {
        GIL_Holder oHolder(false);
        Py_DecRef(m_poConnection);
        m_poConnection = nullptr;
    }

    if (m_poOGRPyGetArrowStream)
    {
        GIL_Holder oHolder(false);
        Py_DecRef(m_poOGRPyGetArrowStream);
        m_poOGRPyGetArrowStream = nullptr;
        Py_DecRef(m_poOGRPyConnect);
        m_poOGRPyConnect = nullptr;
    }
}

std::unique_ptr<GDALDataset> OGRPyDuckDBDataset::CreateMemLayer()
{
    auto poMemDrv = GetGDALDriverManager()->GetDriverByName("Memory");
    if (!poMemDrv)
        return nullptr;
    auto poDS = std::unique_ptr<GDALDataset>(
        poMemDrv->Create("", 0, 0, 0, GDT_Unknown, nullptr));
    if (!poDS)
        return nullptr;
    auto poMemLayer = poDS->CreateLayer("test", nullptr, wkbNone, nullptr);
    if (!poMemLayer)
        return nullptr;

    struct ArrowSchema sSchema;
    memset(&sSchema, 0, sizeof(sSchema));
    if (m_psArrowArrayStream->get_schema(m_psArrowArrayStream, &sSchema) == 0)
    {
        std::string osErrorMsg;
        if (poMemLayer->IsArrowSchemaSupported(&sSchema, nullptr, osErrorMsg))
        {
            // Create output fields using CreateFieldFromArrowSchema()
            for (int i = 0; i < sSchema.n_children; ++i)
            {
                const char *pszFieldName = sSchema.children[i]->name;

                if (!poMemLayer->CreateFieldFromArrowSchema(sSchema.children[i],
                                                            nullptr))
                {
                    CPLError(CE_Warning, CPLE_AppDefined,
                             "Cannot create field %s", pszFieldName);
                }
            }
        }
        else
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Arrow Schema not supported: %s", osErrorMsg.c_str());
            poDS.reset();
        }
        sSchema.release(&sSchema);
    }

    return poDS;
}

void OGRPyDuckDBDataset::ResetArrowArrayStream()
{
    if (m_poCapsule)
    {
        if (m_psArrowArrayStream && m_psArrowArrayStream->release)
        {
            m_psArrowArrayStream->release(m_psArrowArrayStream);
        }
        m_psArrowArrayStream = nullptr;

        GIL_Holder oHolder(false);
        Py_DecRef(m_poCapsule);
        m_poCapsule = nullptr;
    }
}

bool OGRPyDuckDBDataset::InitializeDuckDB()
{
    if (m_poOGRPyGetArrowStream)
        return true;

    CPLDebug("OGRPyDuckDB", "InitializeDuckDB() start");

    // We need to build a unique module name, otherwise this will crash in
    // multithreaded use cases.
    const CPLString osModuleName(CPLSPrintf("ogr_py_duckdb_%p", this));

    const char *pszDuckDBSetup =
        "import duckdb\n"
        "\n"
        "assert [int(x) for x in duckdb.__version__.split('.')[0:2]] >= [1, "
        "1], "
        "f'DuckDB {duckdb.__version__} found, but at least 1.1 expected'\n"
        "\n"
        "def is_capsule(o):\n"
        "   t = type(o)\n"
        "   return t.__module__ == 'builtins' and t.__name__ == 'PyCapsule'\n"
        "\n"
        "def OGRPyConnect(filename, nthreads):\n"
        "    conn = duckdb.connect(filename)\n"
        "    conn.execute(f'SET threads TO {nthreads}')\n"
        //"    conn.execute(\"SET memory_limit = '1GB'\")\n"
        "    conn.execute('SET enable_progress_bar = false')\n"
        "    conn.execute('SET autoinstall_known_extensions = false')\n"
        "    conn.execute('SET autoload_known_extensions = false')\n"
        "    return conn\n"
        "\n"
        "def OGRPyGetArrowStream(conn, sql):\n"
        //"    capsule = conn.execute(sql).fetch_record_batch(65536).__arrow_c_stream__()\n"
        "    capsule = conn.sql(sql).__arrow_c_stream__()\n"
        "    assert is_capsule(capsule)\n"
        "    return capsule\n";

#if 0
    const char* pszPolarsSetup =
        "import polars\n"
        "\n"
        "def is_capsule(o):\n"
        "   t = type(o)\n"
        "   return t.__module__ == 'builtins' and t.__name__ == 'PyCapsule'\n"
        "\n"
        "def OGRPyConnect(filename, nthreads):\n"
        "    conn = polars.read_parquet(filename)\n"
        "    return conn\n"
        "\n"
        "def OGRPyGetArrowStream(conn, sql):\n"
        "    capsule = conn.__arrow_c_stream__()\n"
        "    assert is_capsule(capsule)\n"
        "    return capsule\n";
#endif

    PyObject *poCompiledString =
        Py_CompileString(pszDuckDBSetup, osModuleName, Py_file_input);
    if (!poCompiledString || PyErr_Occurred())
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Couldn't compile code:\n%s",
                 GetPyExceptionString().c_str());
        return false;
    }
    PyObject *poModule =
        PyImport_ExecCodeModule(osModuleName, poCompiledString);
    Py_DecRef(poCompiledString);

    if (!poModule || PyErr_Occurred())
    {
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 GetPyExceptionString().c_str());
        return false;
    }

    // Fetch our OGRPyConnect python function
    auto poOGRPyConnect = PyObject_GetAttrString(poModule, "OGRPyConnect");
    if (!poOGRPyConnect || PyErr_Occurred())
    {
        // Shouldn't happen normally...
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 GetPyExceptionString().c_str());
        Py_DecRef(poModule);
        return false;
    }

    // Fetch our OGRPyGetArrowStream python function
    auto poOGRPyGetArrowStream =
        PyObject_GetAttrString(poModule, "OGRPyGetArrowStream");
    if (!poOGRPyGetArrowStream || PyErr_Occurred())
    {
        // Shouldn't happen normally...
        CPLError(CE_Failure, CPLE_AppDefined, "%s",
                 GetPyExceptionString().c_str());
        Py_DecRef(poOGRPyConnect);
        Py_DecRef(poModule);
        return false;
    }
    Py_DecRef(poModule);

    CPLDebug("OGRPyDuckDB", "InitializeDuckDB() end");

    m_poOGRPyGetArrowStream = poOGRPyGetArrowStream;
    m_poOGRPyConnect = poOGRPyConnect;
    return true;
}

bool OGRPyDuckDBDataset::Connect()
{
    GIL_Holder oHolder(false);

    if (!InitializeDuckDB())
        return false;

    PyObject *pyArgs = PyTuple_New(2);
    PyTuple_SetItem(pyArgs, 0,
                    PyUnicode_FromString(m_osConnectionName.c_str()));
    const char *pszNumThreads =
        CPLGetConfigOption("GDAL_NUM_THREADS", "ALL_CPUS");
    const int nThreads =
        std::clamp(EQUAL(pszNumThreads, "ALL_CPUS") ? CPLGetNumCPUs()
                                                    : atoi(pszNumThreads),
                   1, 1024);
    PyTuple_SetItem(pyArgs, 1, PyLong_FromLong(nThreads));

    // Call OGRPyConnect
    PyObject *pRetValue = PyObject_Call(m_poOGRPyConnect, pyArgs, nullptr);

    Py_DecRef(pyArgs);

    if (ErrOccurredEmitCPLError() || !pRetValue)
    {
        return false;
    }

    m_poConnection = pRetValue;
    return true;
}

bool OGRPyDuckDBDataset::GetArrowArrayStream()
{
    ResetArrowArrayStream();

    GIL_Holder oHolder(false);

    PyObject *pyArgs = PyTuple_New(2);
    Py_IncRef(m_poConnection);
    PyTuple_SetItem(pyArgs, 0, m_poConnection);
    PyTuple_SetItem(pyArgs, 1, PyUnicode_FromString(m_osSQL.c_str()));

    // Call OGRPyGetArrowStream
    PyObject *poCapsule = nullptr;
    try
    {
        poCapsule = PyObject_Call(m_poOGRPyGetArrowStream, pyArgs, nullptr);
    }
    catch (const std::exception &e)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Uncaught C++ exception in OGRPyGetArrowStream(): %s",
                 e.what());
        Py_DecRef(pyArgs);
        return false;
    }
    catch (...)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Uncaught C++ exception in OGRPyGetArrowStream()");
        Py_DecRef(pyArgs);
        return false;
    }

    Py_DecRef(pyArgs);

    if (ErrOccurredEmitCPLError() || !poCapsule)
    {
        return false;
    }

    // Extract the C ArrowArrayStream from the PyCapsule
    auto psArrowArrayStream = static_cast<ArrowArrayStream *>(
        PyCapsule_GetPointer(poCapsule, "arrow_array_stream"));
    if (!psArrowArrayStream || ErrOccurredEmitCPLError())
    {
        Py_DecRef(poCapsule);
        return false;
    }

    // We need to keep a reference on the PyCapsule, otherwise the
    // C ArrowArrayStream gets invalidated (in theory that should not be
    // the case, but the DuckDB Python wrapper has probably not being
    // nominally designed to fully hand-over an ArrowArrayStream to a non-Python
    // consumer)
    m_poCapsule = poCapsule;
    m_psArrowArrayStream = psArrowArrayStream;
    return true;
}

/*static*/
std::unique_ptr<OGRPyDuckDBDataset> OGRPyDuckDBDataset::Create(
    const std::string &osConnectionName, OGRPyDuckDBDataset *poMainDS,
    const std::string &osFilename, const std::string &osSQL)
{
    auto poDS = std::make_unique<OGRPyDuckDBDataset>();
    poDS->m_osConnectionName = osConnectionName;
    if (poMainDS)
    {
        poDS->m_poConnection = poMainDS->m_poConnection;
        poDS->m_poOGRPyGetArrowStream = poMainDS->m_poOGRPyGetArrowStream;
        poDS->m_poOGRPyConnect = poMainDS->m_poOGRPyConnect;
        GIL_Holder oHolder(false);
        Py_IncRef(poDS->m_poConnection);
        Py_IncRef(poDS->m_poOGRPyGetArrowStream);
        Py_IncRef(poDS->m_poOGRPyConnect);
    }
    else
    {
        if (!poDS->Connect())
            return nullptr;
    }
    poDS->m_osFilename = osFilename;
    poDS->m_osSQL = osSQL;
    if (!poDS->GetArrowArrayStream())
        return nullptr;
    auto poLayer = std::make_unique<OGRPyDuckDBLayer>(poDS.get());
    if (!poLayer->m_poMemDS)
        return nullptr;
    poDS->m_poLayer = std::move(poLayer);
    return poDS;
}

/* static */
GDALDataset *OGRPyDuckDBDataset::Open(GDALOpenInfo *poOpenInfo)
{
    if (poOpenInfo->eAccess == GA_Update)
        return nullptr;
    if (!STARTS_WITH_CI(poOpenInfo->pszFilename, "PyDuckDB:"))
        return nullptr;
    if (!GDALPythonInitialize())
        return nullptr;

    const char *pszSQLOrFilename =
        poOpenInfo->pszFilename + strlen("PyDuckDB:");
    if (STARTS_WITH_CI(pszSQLOrFilename, "SELECT ") ||
        STARTS_WITH_CI(pszSQLOrFilename, "LOAD ") ||
        STARTS_WITH_CI(pszSQLOrFilename, "DESCRIBE ") ||
        STARTS_WITH_CI(pszSQLOrFilename, "SUMMARIZE "))
    {
        return Create(":memory:", nullptr, std::string(), pszSQLOrFilename)
            .release();
    }
    else if (EQUAL(CPLGetExtension(pszSQLOrFilename), "parquet"))
    {
        const auto osSQL =
            std::string("SELECT * FROM '")
                .append(CPLString(pszSQLOrFilename).replaceAll("'", "''"))
                .append("'");
        return Create(":memory", nullptr, pszSQLOrFilename, osSQL).release();
    }
    else
    {
        auto poDSTablesList =
            Create(pszSQLOrFilename, nullptr, pszSQLOrFilename, "SHOW TABLES");
        if (poDSTablesList)
        {
            auto poLyrTablesList = poDSTablesList->GetLayer(0);
            for (auto &&f : poLyrTablesList)
            {
                const auto osSQL = std::string("SELECT * FROM \"")
                                       .append(CPLString(f->GetFieldAsString(0))
                                                   .replaceAll("\"", "\"\""))
                                       .append("\"");
                return Create(pszSQLOrFilename, poDSTablesList.get(),
                              pszSQLOrFilename, osSQL)
                    .release();
            }
        }
        return nullptr;
    }
}

/************************************************************************/
/*                        RegisterOGRPyDuckDB()                         */
/************************************************************************/

void RegisterOGRPyDuckDB()
{
    if (GDALGetDriverByName(DRIVER_NAME) != nullptr)
        return;

    const char *const apszArgv[] = {
        "python3", "-c",
        "import importlib.util; "
        "import sys; "
        "sys.exit(0 if importlib.util.find_spec('duckdb') else 1)",
        nullptr};
    CPLDebugOnly("OGRPyDuckDB",
                 "Start detecting if there is a duckdb Python module...");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    const int nRetCode = CPLSpawn(apszArgv, nullptr, nullptr, false);
    CPLPopErrorHandler();
    if (nRetCode == 0)
    {
        CPLDebugOnly("OGRPyDuckDB", " --> yes there is");
    }
    else if (nRetCode == 1)
    {
        CPLDebugOnly("OGRPyDuckDB", " --> nope");
        return;
    }
    else
    {
        CPLDebugOnly("OGRPyDuckDB", " --> could not find python3");
    }

    auto poDriver = std::make_unique<GDALDriver>();
    poDriver->SetDescription(DRIVER_NAME);
    poDriver->SetMetadataItem(GDAL_DCAP_VECTOR, "YES");

    poDriver->pfnOpen = OGRPyDuckDBDataset::Open;

    GetGDALDriverManager()->RegisterDriver(poDriver.release());
}
