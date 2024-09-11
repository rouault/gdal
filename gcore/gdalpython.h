/******************************************************************************
 *
 * Project:  GDAL Core
 * Purpose:  Python interface
 * Author:   Even Rouault, <even dot rouault at spatialys dot com>
 *
 ******************************************************************************
 * Copyright (c) 2017-2019, Even Rouault, <even dot rouault at spatialys dot
 *com>
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

#ifndef GDALPYTHON_H_INCLUDED
#define GDALPYTHON_H_INCLUDED

#include "cpl_string.h"
#include <cwchar>

bool CPL_DLL GDALPythonInitialize();

void GDALPythonFinalize();

//! @cond Doxygen_Suppress

// Subset of Python API defined as function pointers
// Only use the below function pointers if GDALPythonInitialize() succeeds
namespace GDALPy
{
typedef struct _object PyObject;
typedef size_t Py_ssize_t;

extern int CPL_DLL (*Py_IsInitialized)(void);
extern void CPL_DLL (*Py_SetProgramName)(const wchar_t *);
extern void CPL_DLL (*Py_SetPythonHome)(const wchar_t *);
extern PyObject CPL_DLL *(*PyObject_Type)(PyObject *);
extern int CPL_DLL (*PyObject_IsInstance)(PyObject *, PyObject *);
extern PyObject CPL_DLL *(*PyTuple_New)(size_t);
// extern size_t CPL_DLL (*PyTuple_Size)(PyObject*);
// extern PyObject CPL_DLL *(*PyTuple_GetItem)(PyObject*, size_t);
extern PyObject CPL_DLL *(*PyBool_FromLong)(long);
extern PyObject CPL_DLL *(*PyLong_FromLong)(long);
extern long CPL_DLL (*PyLong_AsLong)(PyObject *);
extern PyObject CPL_DLL *(*PyLong_FromLongLong)(GIntBig);
extern GIntBig CPL_DLL (*PyLong_AsLongLong)(PyObject *);
extern PyObject CPL_DLL *(*PyFloat_FromDouble)(double);
extern double CPL_DLL (*PyFloat_AsDouble)(PyObject *);
extern PyObject CPL_DLL *(*PyObject_Call)(PyObject *, PyObject *, PyObject *);
extern PyObject CPL_DLL *(*PyObject_GetIter)(PyObject *);
extern PyObject CPL_DLL *(*PyIter_Next)(PyObject *);
extern void CPL_DLL (*Py_IncRef)(PyObject *);
extern void CPL_DLL (*Py_DecRef)(PyObject *);
extern PyObject CPL_DLL *(*PyErr_Occurred)(void);
extern void CPL_DLL (*PyErr_Print)(void);

extern PyObject CPL_DLL *(*Py_CompileString)(const char *, const char *, int);
extern PyObject CPL_DLL *(*PyImport_ExecCodeModule)(const char *, PyObject *);
extern int CPL_DLL (*PyObject_HasAttrString)(PyObject *, const char *);
extern PyObject CPL_DLL *(*PyObject_GetAttrString)(PyObject *, const char *);
extern int CPL_DLL (*PyObject_SetAttrString)(PyObject *, const char *,
                                             PyObject *);
extern int CPL_DLL (*PyTuple_SetItem)(PyObject *, size_t, PyObject *);
extern void CPL_DLL (*PyObject_Print)(PyObject *, FILE *, int);

extern Py_ssize_t CPL_DLL (*PyBytes_Size)(PyObject *);
extern const char CPL_DLL *(*PyBytes_AsString)(PyObject *);
extern int CPL_DLL *(*PyBytes_AsStringAndSize)(PyObject *, char **,
                                               Py_ssize_t *);
extern PyObject CPL_DLL *(*PyBytes_FromObject)(PyObject *);
extern PyObject CPL_DLL *(*PyBytes_FromStringAndSize)(const void *, size_t);

extern PyObject CPL_DLL *(*PyUnicode_FromString)(const char *);
extern PyObject CPL_DLL *(*PyUnicode_AsUTF8String)(PyObject *);
extern PyObject CPL_DLL *(*PyImport_ImportModule)(const char *);
extern int CPL_DLL (*PyCallable_Check)(PyObject *);
extern PyObject CPL_DLL *(*PyDict_New)(void);
extern int CPL_DLL (*PyDict_SetItemString)(PyObject *p, const char *key,
                                           PyObject *val);
extern int CPL_DLL (*PyDict_Next)(PyObject *p, size_t *, PyObject **,
                                  PyObject **);
extern PyObject CPL_DLL *(*PyDict_GetItemString)(PyObject *p, const char *key);
extern PyObject CPL_DLL *(*PyList_New)(Py_ssize_t);
extern int CPL_DLL (*PyList_SetItem)(PyObject *, Py_ssize_t, PyObject *);
extern int CPL_DLL (*PyArg_ParseTuple)(PyObject *, const char *, ...);

extern int CPL_DLL (*PySequence_Check)(PyObject *o);
extern Py_ssize_t CPL_DLL (*PySequence_Size)(PyObject *o);
extern PyObject CPL_DLL *(*PySequence_GetItem)(PyObject *o, Py_ssize_t i);

extern void CPL_DLL (*PyErr_Fetch)(PyObject **poPyType, PyObject **poPyValue,
                                   PyObject **poPyTraceback);
extern void CPL_DLL (*PyErr_Clear)(void);
extern const char CPL_DLL *(*Py_GetVersion)(void);

extern void CPL_DLL *(*PyCapsule_GetPointer)(PyObject *, const char *);

typedef struct
{
    // cppcheck-suppress unusedStructMember
    char big_enough[256];
} Py_buffer;

extern int CPL_DLL (*PyBuffer_FillInfo)(Py_buffer *view, PyObject *obj,
                                        void *buf, size_t len, int readonly,
                                        int infoflags);
extern PyObject CPL_DLL *(*PyMemoryView_FromBuffer)(Py_buffer *view);

typedef PyObject *(*PyCFunction)(PyObject *, PyObject *, PyObject *);

typedef struct PyMethodDef PyMethodDef;

struct PyMethodDef
{
    const char *name;
    PyCFunction function;
    int flags;
    const char *help;
};

extern PyObject CPL_DLL *(*PyModule_Create2)(struct PyModuleDef *, int);

#define PYTHON_API_VERSION 1013

/* Flag passed to newmethodobject */
#define METH_VARARGS 0x0001
#define METH_KEYWORDS 0x0002

#define _PyObject_HEAD_EXTRA

struct _object
{
    _PyObject_HEAD_EXTRA Py_ssize_t ob_refcnt;
    void /*struct _typeobject*/ *ob_type;
};

#define PyObject_HEAD PyObject ob_base;

#define _PyObject_EXTRA_INIT

#define PyObject_HEAD_INIT(type) {_PyObject_EXTRA_INIT 1, type},

#define PyModuleDef_HEAD_INIT                                                  \
    {                                                                          \
        PyObject_HEAD_INIT(nullptr) nullptr, /* m_init */                      \
            0,                               /* m_index */                     \
            nullptr,                         /* m_copy */                      \
    }

typedef struct PyModuleDef_Base
{
    PyObject_HEAD PyObject *(*m_init)(void);
    Py_ssize_t m_index;
    PyObject *m_copy;
} PyModuleDef_Base;

typedef void *traverseproc;
typedef void *inquiry;
typedef void *freefunc;

typedef struct PyModuleDef
{
    PyModuleDef_Base m_base;
    const char *m_name;
    const char *m_doc;
    Py_ssize_t m_size;
    const PyMethodDef *m_methods;
    struct PyModuleDef_Slot *m_slots;
    traverseproc m_traverse;
    inquiry m_clear;
    freefunc m_free;
} PyModuleDef;

#define Py_file_input 257

typedef int PyGILState_STATE;

class CPL_DLL GIL_Holder
{
    const bool m_bExclusiveLock;
    PyGILState_STATE m_eState = 0;

  public:
    explicit GIL_Holder(bool bExclusiveLock);
    virtual ~GIL_Holder();
};

CPLString CPL_DLL GetString(PyObject *obj, bool bEmitError = true);
CPLString CPL_DLL GetPyExceptionString();
bool CPL_DLL ErrOccurredEmitCPLError();

}  // namespace GDALPy

//! @endcond

#endif
