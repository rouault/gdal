#!/usr/bin/env python
# -*- coding: utf-8 -*-
###############################################################################
# $Id$
#
# Project:  GDAL/OGR Test Suite
# Purpose:  GMLAS driver testing.
# Author:   Even Rouault, <even dot rouault at spatialys dot com>
#
# Initial development funded by the European Earth observation programme
# Copernicus
#
#******************************************************************************
# Copyright (c) 2016, Even Rouault, <even dot rouault at spatialys dot com>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
###############################################################################

import os
import sys

sys.path.append( '../pymod' )

import gdaltest
import ogrtest
from osgeo import gdal
from osgeo import ogr

###############################################################################
# Basic test

def ogr_gmlas_basic():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    ds = ogr.Open('GMLAS:data/gmlas_test1.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'
    ds = None

    import test_cli_utilities

    if test_cli_utilities.get_ogrinfo_path() is None:
        return 'skip'

    ret = gdaltest.runexternal(test_cli_utilities.get_ogrinfo_path() + ' -ro -al GMLAS:data/gmlas_test1.xml')
    ret = ret.replace('\r\n', '\n')
    expected = open('data/gmlas_test1.txt', 'rb').read().decode('utf-8')
    expected = expected.replace('\r\n', '\n')
    if ret != expected:
        gdaltest.post_reason('fail')
        print('Got:')
        print(ret)
        open('tmp/ogr_gmlas_1.txt', 'wb').write(ret.encode('utf-8'))
        print('Diff:')
        os.system('diff -u data/gmlas_test1.txt tmp/ogr_gmlas_1.txt')
        os.unlink('tmp/ogr_gmlas_1.txt')
        return 'fail'

    return 'success'

###############################################################################
# Test virtual file support

def ogr_gmlas_virtual_file():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_8.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_8.xsd"/>""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_8.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="main_elt" type="xs:string"/>
</xs:schema>""")


    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_8.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_8.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_8.xsd')

    return 'success'

###############################################################################
# Test opening with just XSD option

def ogr_gmlas_no_datafile_with_xsd_option():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    ds = gdal.OpenEx('GMLAS:', open_options = ['XSD=data/gmlas_test1.xsd'])
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'

    return 'success'

###############################################################################
# Test opening with just XSD option but pointing to a non-xsd filename

def ogr_gmlas_no_datafile_xsd_which_is_not_xsd():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:', open_options = ['XSD=data/gmlas_test1.xml'])
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find("invalid content in 'schema' element") < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    return 'success'

###############################################################################
# Test opening with nothing

def ogr_gmlas_no_datafile_no_xsd():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:')
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('XSD open option must be provided when no XML data file is passed') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    return 'success'

###############################################################################
# Test opening an inexisting GML file

def ogr_gmlas_non_existing_gml():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:/vsimem/i_dont_exist.gml')
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('Cannot open /vsimem/i_dont_exist.gml') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    return 'success'

###############################################################################
# Test opening with just XSD option but pointing to a non existing file

def ogr_gmlas_non_existing_xsd():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:', open_options = ['XSD=/vsimem/i_dont_exist.xsd'])
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('Cannot resolve /vsimem/i_dont_exist.xsd') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    return 'success'

###############################################################################
# Test opening a GML file without schemaLocation

def ogr_gmlas_gml_without_schema_location():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_gml_without_schema_location.xml',
"""<MYNS:main_elt xmlns:MYNS="http://myns"/>""")

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_gml_without_schema_location.xml')
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('No schema locations found when analyzing data file: XSD open option must be provided') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_gml_without_schema_location.xml')

    return 'success'

###############################################################################
# Test invalid schema

def ogr_gmlas_invalid_schema():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_invalid_schema.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_invalid_schema.xsd"/>""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_invalid_schema.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:foo/>
</xs:schema>""")

    with gdaltest.error_handler():
        ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_invalid_schema.xml')
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('invalid content') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_invalid_schema.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_invalid_schema.xsd')

    return 'success'

###############################################################################
# Test invalid XML

def ogr_gmlas_invalid_xml():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_invalid_xml.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_invalid_xml.xsd">
""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_invalid_xml.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="main_elt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="foo" type="xs:string" minOccurs="0"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_invalid_xml.xml')
    lyr = ds.GetLayer(0)
    with gdaltest.error_handler():
        f = lyr.GetNextFeature()
    if f is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if gdal.GetLastErrorMsg().find('input ended before all started tags were ended') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_invalid_xml.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_invalid_xml.xsd')

    return 'success'

###############################################################################
# Test links with gml:ReferenceType

def ogr_gmlas_gml_Reference():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    ds = ogr.Open('GMLAS:data/gmlas_test_targetelement.xml')
    if ds.GetLayerCount() != 2:
        gdaltest.post_reason('fail')
        print(ds.GetLayerCount())
        return 'fail'

    lyr = ds.GetLayerByName('main_elt')
    f = lyr.GetNextFeature()
    if f['reference_existing_target_elt_href'] != '#BAZ' or \
       f['reference_existing_target_elt_pkid'] != 'BAZ' or \
       f['reference_existing_abstract_target_elt_href'] != '#BAW':
           gdaltest.post_reason('fail')
           f.DumpReadable()
           return 'fail'

    return 'success'

###############################################################################
# Test that we fix ambiguities in class names

def ogr_gmlas_same_element_in_different_ns():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_same_element_in_different_ns.xml',
"""<myns:elt xmlns:myns="http://myns"
             xmlns:other_ns="http://other_ns" 
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_same_element_in_different_ns.xsd">
    <other_ns:realizationOfAbstractElt>
        <other_ns:foo>bar</other_ns:foo>
    </other_ns:realizationOfAbstractElt>
</myns:elt>""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_same_element_in_different_ns.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns"
           xmlns:other_ns="http://other_ns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:import namespace="http://other_ns" schemaLocation="ogr_gmlas_same_element_in_different_ns_other_ns.xsd"/>
<xs:element name="elt">
  <xs:complexType>
    <xs:sequence>
        <xs:element ref="other_ns:abstractElt"/>
        <xs:element name="elt2">
            <xs:complexType>
                <xs:sequence>
                    <xs:element ref="other_ns:abstractElt" maxOccurs="unbounded"/>
                </xs:sequence>
            </xs:complexType>
        </xs:element>
    </xs:sequence>
  </xs:complexType>
</xs:element>
<xs:element name="realizationOfAbstractElt" substitutionGroup="other_ns:abstractElt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="bar" type="xs:string"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_same_element_in_different_ns_other_ns.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:other_ns="http://other_ns" 
           targetNamespace="http://other_ns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="abstractElt" abstract="true"/>
<xs:element name="realizationOfAbstractElt" substitutionGroup="other_ns:abstractElt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="foo" type="xs:string"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_same_element_in_different_ns.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'
    #for i in range(ds.GetLayerCount()):
    #    print(ds.GetLayer(i).GetName())

    if ds.GetLayerCount() != 5:
        gdaltest.post_reason('fail')
        print(ds.GetLayerCount())
        return 'fail'
    lyr = ds.GetLayerByName('elt')
    f = lyr.GetNextFeature()
    if f.IsFieldSet('abstractElt_other_ns_realizationOfAbstractElt_pkid') == 0:
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if ds.GetLayerByName('myns_realizationOfAbstractElt') is None:
        gdaltest.post_reason('fail')
        return 'fail'
    if ds.GetLayerByName('other_ns_realizationOfAbstractElt') is None:
        gdaltest.post_reason('fail')
        return 'fail'
    if ds.GetLayerByName('elt_elt2_abstractElt_myns_realizationOfAbstractElt') is None:
        gdaltest.post_reason('fail')
        return 'fail'
    if ds.GetLayerByName('elt_elt2_abstractElt_other_ns_realizationOfAbstractElt') is None:
        gdaltest.post_reason('fail')
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_same_element_in_different_ns.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_same_element_in_different_ns.xsd')
    gdal.Unlink('/vsimem/ogr_gmlas_same_element_in_different_ns_other_ns.xsd')

    return 'success'

###############################################################################
# Test a corner case of relative path resolution

def ogr_gmlas_corner_case_relative_path():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    ds = ogr.Open('GMLAS:../ogr/data/gmlas_test1.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'

    return 'success'

###############################################################################
# Test unexpected repeated element

def ogr_gmlas_unexpected_repeated_element():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_unexpected_repeated_element.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_unexpected_repeated_element.xsd">
    <myns:foo>foo_first</myns:foo>
    <myns:foo>foo_again</myns:foo>
</myns:main_elt>
""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_unexpected_repeated_element.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="main_elt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="foo" type="xs:string" minOccurs="0"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_unexpected_repeated_element.xml')
    lyr = ds.GetLayer(0)
    with gdaltest.error_handler():
        f = lyr.GetNextFeature()
    if f is None or f['foo'] != 'foo_again': # somewhat arbitrary to keep the latest one!
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if gdal.GetLastErrorMsg().find('Unexpected element myns:main_elt/myns:foo') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'
    f = lyr.GetNextFeature()
    if f is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    ds = None

    gdal.Unlink('/vsimem/ogr_gmlas_unexpected_repeated_element.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_unexpected_repeated_element.xsd')

    return 'success'

###############################################################################
# Test unexpected repeated element

def ogr_gmlas_unexpected_repeated_element_variant():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_unexpected_repeated_element.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_unexpected_repeated_element.xsd">
    <myns:foo>foo_first</myns:foo>
    <myns:bar>bar</myns:bar>
    <myns:foo>foo_again</myns:foo>
</myns:main_elt>
""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_unexpected_repeated_element.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="main_elt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="foo" type="xs:string" minOccurs="0"/>
        <xs:element name="bar" type="xs:string" minOccurs="0"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_unexpected_repeated_element.xml')
    lyr = ds.GetLayer(0)
    with gdaltest.error_handler():
        f = lyr.GetNextFeature()
    if f is None or f['foo'] != 'foo_again': # somewhat arbitrary to keep the latest one!
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if gdal.GetLastErrorMsg().find('Unexpected element myns:main_elt/myns:foo') < 0:
        gdaltest.post_reason('fail')
        print(gdal.GetLastErrorMsg())
        return 'fail'
    f = lyr.GetNextFeature()
    if f is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    ds = None

    gdal.Unlink('/vsimem/ogr_gmlas_unexpected_repeated_element.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_unexpected_repeated_element.xsd')

    return 'success'

###############################################################################
# Test reading geometries embedded in a geometry property element

def ogr_gmlas_geometryproperty():

    ds = gdal.OpenEx('GMLAS:data/gmlas_geometryproperty_gml32.gml')
    lyr = ds.GetLayer(0)
    with gdaltest.error_handler():
        geom_field_count = lyr.GetLayerDefn().GetGeomFieldCount()
    if geom_field_count != 14:
        gdaltest.post_reason('fail')
        print(geom_field_count)
        return 'fail'
    f = lyr.GetNextFeature()
    if f['geometryProperty_xml'] != ' <gml:Point gml:id="poly.geom.Geometry" srsName="urn:ogc:def:crs:EPSG::4326"> <gml:pos>49 2</gml:pos> </gml:Point> ':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if f.IsFieldSet('geometryPropertyEmpty_xml'):
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if f['pointProperty_xml'] != '<gml:Point gml:id="poly.geom.Point" srsName="urn:ogc:def:crs:EPSG::4326"><gml:pos>50 3</gml:pos></gml:Point>':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if f['pointPropertyRepeated_xml'] != [
            '<gml:Point gml:id="poly.geom.pointPropertyRepeated.1"><gml:pos>0 1</gml:pos></gml:Point>',
            '<gml:Point gml:id="poly.geom.pointPropertyRepeated.2"><gml:pos>1 2</gml:pos></gml:Point>',
            '<gml:Point gml:id="poly.geom.pointPropertyRepeated.3"><gml:pos>3 4</gml:pos></gml:Point>']:
        gdaltest.post_reason('fail')
        print(f['pointPropertyRepeated_xml'])
        f.DumpReadable()
        return 'fail'
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('geometryProperty')
    sr = lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetSpatialRef()
    if sr is None or sr.ExportToWkt().find('4326') < 0 or sr.ExportToWkt().find('AXIS') >= 0:
        gdaltest.post_reason('fail')
        print(sr)
        return 'fail'
    wkt = f.GetGeomFieldRef(geom_idx).ExportToWkt()
    # Axis swapping
    if wkt != 'POINT (2 49)':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('geometryPropertyEmpty')
    if f.GetGeomFieldRef(geom_idx) is not None:
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('pointProperty')
    sr = lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetSpatialRef()
    if sr is None or sr.ExportToWkt().find('4326') < 0 or sr.ExportToWkt().find('AXIS') >= 0:
        gdaltest.post_reason('fail')
        print(sr)
        return 'fail'
    wkt = f.GetGeomFieldRef(geom_idx).ExportToWkt()
    if wkt != 'POINT (3 50)':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('lineStringProperty')
    sr = lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetSpatialRef()
    if sr is None or sr.ExportToWkt().find('4326') < 0 or sr.ExportToWkt().find('AXIS') >= 0:
        gdaltest.post_reason('fail')
        print(sr)
        return 'fail'
    if lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetType() != ogr.wkbLineString:
        gdaltest.post_reason('fail')
        print(lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetType())
        return 'fail'
    wkt = f.GetGeomFieldRef(geom_idx).ExportToWkt()
    if wkt != 'LINESTRING (2 49)':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('pointPropertyRepeated')
    if lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetType() != ogr.wkbUnknown:
        gdaltest.post_reason('fail')
        print(lyr.GetLayerDefn().GetGeomFieldDefn(geom_idx).GetType())
        return 'fail'
    wkt = f.GetGeomFieldRef(geom_idx).ExportToWkt()
    if wkt != 'GEOMETRYCOLLECTION (POINT (0 1),POINT (1 2),POINT (3 4))':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'

    # Test that on-the-fly reprojection works
    f = lyr.GetNextFeature()
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('geometryProperty')
    geom = f.GetGeomFieldRef(geom_idx)
    if ogrtest.check_feature_geometry(geom, 'POINT (3.0 0.0)') != 0:
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'

    # Failed reprojection
    with gdaltest.error_handler():
        f = lyr.GetNextFeature()
    geom_idx = lyr.GetLayerDefn().GetGeomFieldIndex('geometryProperty')
    if f.GetGeomFieldRef(geom_idx) is not None:
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'

    return 'success'

###############################################################################
# Test reading geometries referenced by a AbstractGeometry element

def ogr_gmlas_abstractgeometry():

    ds = gdal.OpenEx('GMLAS:data/gmlas_abstractgeometry_gml32.gml')
    lyr = ds.GetLayer(0)
    if lyr.GetLayerDefn().GetGeomFieldCount() != 2:
        gdaltest.post_reason('fail')
        print(lyr.GetLayerDefn().GetGeomFieldCount())
        return 'fail'
    f = lyr.GetNextFeature()
    if f['AbstractGeometry_xml'] != '<gml:Point gml:id="test.geom.0"><gml:pos>0 1</gml:pos></gml:Point>':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    if f['repeated_AbstractGeometry_xml'] != [
            '<gml:Point gml:id="test.geom.repeated.1"><gml:pos>0 1</gml:pos>',
            '<gml:Point gml:id="test.geom.repeated.2"><gml:pos>1 2</gml:pos>']:
        gdaltest.post_reason('fail')
        print(f['repeated_AbstractGeometry_xml'])
        f.DumpReadable()
        return 'fail'
    wkt = f.GetGeomFieldRef(0).ExportToWkt()
    if wkt != 'POINT (0 1)':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'
    wkt = f.GetGeomFieldRef(1).ExportToWkt()
    if wkt != 'GEOMETRYCOLLECTION (POINT (0 1),POINT (1 2))':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'

    return 'success'

###############################################################################
# Test validation against schema

class MyHandler:
    def __init__(self):
        self.error_list = []

    def error_handler(self, err_type, err_no, err_msg):
        self.error_list.append((err_type, err_no, err_msg))

def ogr_gmlas_validate():

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_validate.xml',
"""<myns:main_elt xmlns:myns="http://myns"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://myns ogr_gmlas_validate.xsd">
    <myns:bar>bar</myns:bar>
</myns:main_elt>
""")

    gdal.FileFromMemBuffer('/vsimem/ogr_gmlas_validate.xsd',
"""<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
           xmlns:myns="http://myns" 
           targetNamespace="http://myns"
           elementFormDefault="qualified" attributeFormDefault="unqualified">
<xs:element name="main_elt">
  <xs:complexType>
    <xs:sequence>
        <xs:element name="foo" type="xs:string"/>
    </xs:sequence>
  </xs:complexType>
</xs:element>
</xs:schema>""")

    # By default check we are silent about validation error
    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_validate.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'
    lyr = ds.GetLayer(0)
    lyr.GetFeatureCount()
    if gdal.GetLastErrorMsg() != '':
        gdaltest.post_reason('fail')
        return 'fail'

    # Enable validation on a doc without validation errors
    myhandler = MyHandler()
    gdal.PushErrorHandler(myhandler.error_handler)
    ds = gdal.OpenEx('GMLAS:data/gmlas_test1.xml', open_options = ['VALIDATE=YES'])
    gdal.PopErrorHandler()
    if ds is None:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'
    if len(myhandler.error_list) != 0:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'

    # Enable validation on a doc without validation error, and with explicit XSD
    gdal.FileFromMemBuffer('/vsimem/gmlas_test1.xml',
                           open('data/gmlas_test1.xml').read() )
    myhandler = MyHandler()
    gdal.PushErrorHandler(myhandler.error_handler)
    ds = gdal.OpenEx('GMLAS:/vsimem/gmlas_test1.xml', open_options = [
                'XSD=' + os.getcwd() + '/data/gmlas_test1.xsd', 'VALIDATE=YES'])
    gdal.PopErrorHandler()
    gdal.Unlink('/vsimem/gmlas_test1.xml')
    if ds is None:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'
    if len(myhandler.error_list) != 0:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'

    # Validation errors, but do not prevent dataset opening
    myhandler = MyHandler()
    gdal.PushErrorHandler(myhandler.error_handler)
    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_validate.xml', open_options = ['VALIDATE=YES'])
    gdal.PopErrorHandler()
    if ds is None:
        gdaltest.post_reason('fail')
        return 'fail'
    if len(myhandler.error_list) != 2:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'

    # Validation errors and do prevent dataset opening
    myhandler = MyHandler()
    gdal.PushErrorHandler(myhandler.error_handler)
    ds = gdal.OpenEx('GMLAS:/vsimem/ogr_gmlas_validate.xml', open_options = ['VALIDATE=YES', 'FAIL_IF_VALIDATION_ERROR=YES'])
    gdal.PopErrorHandler()
    if ds is not None:
        gdaltest.post_reason('fail')
        return 'fail'
    if len(myhandler.error_list) != 3:
        gdaltest.post_reason('fail')
        print(myhandler.error_list)
        return 'fail'

    gdal.Unlink('/vsimem/ogr_gmlas_validate.xml')
    gdal.Unlink('/vsimem/ogr_gmlas_validate.xsd')

    return 'success'

###############################################################################
# Test correct namespace prefix handling

def ogr_gmlas_test_ns_prefix():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    # The schema doesn't directly import xlink, but indirectly references it
    ds = gdal.OpenEx('GMLAS:', open_options = ['XSD=data/gmlas_test_targetelement.xsd'])

    lyr = ds.GetLayerByName('_ogr_fields_metadata')
    f = lyr.GetNextFeature()
    if f['field_xpath'] != 'myns:main_elt/myns:reference_missing_target_elt@xlink:href':
        gdaltest.post_reason('fail')
        f.DumpReadable()
        return 'fail'

    return 'success'


###############################################################################
#  Cleanup

def ogr_gmlas_cleanup():

    if ogr.GetDriverByName('GMLAS') is None:
        return 'skip'

    return 'success'


gdaltest_list = [
    ogr_gmlas_basic,
    ogr_gmlas_virtual_file,
    ogr_gmlas_no_datafile_with_xsd_option,
    ogr_gmlas_no_datafile_xsd_which_is_not_xsd,
    ogr_gmlas_no_datafile_no_xsd,
    ogr_gmlas_non_existing_gml,
    ogr_gmlas_non_existing_xsd,
    ogr_gmlas_gml_without_schema_location,
    ogr_gmlas_invalid_schema,
    ogr_gmlas_invalid_xml,
    ogr_gmlas_gml_Reference,
    ogr_gmlas_same_element_in_different_ns,
    ogr_gmlas_corner_case_relative_path,
    ogr_gmlas_unexpected_repeated_element,
    ogr_gmlas_unexpected_repeated_element_variant,
    ogr_gmlas_geometryproperty,
    ogr_gmlas_abstractgeometry,
    ogr_gmlas_validate,
    ogr_gmlas_test_ns_prefix,
    ogr_gmlas_cleanup ]

if __name__ == '__main__':

    gdaltest.setup_run( 'ogr_gmlas' )

    gdaltest.run_tests( gdaltest_list )

    gdaltest.summarize()
