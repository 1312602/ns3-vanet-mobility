## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
#    obj = bld.create_ns3_program('vanet-highway-test', ['core'])
#    obj.source = [
#	'vanet-highway-test.cc',
#	'Highway.cc',
#	'Controller.cc',
#	'Vehicle.cc',
#	'Obstacle.cc',
#	'Model.cc',
#	'LaneChange.cc',
#	]

    obj = bld.create_ns3_program('101112-2rsu1vehicle', ['core'])
    obj.source = [
	'101112-2rsu1vehicle.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]


    obj = bld.create_ns3_program('101202-fixedvaluesim', ['core'])
    obj.source = [
	'101202-fixedvaluesim.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]

    obj = bld.create_ns3_program('101202-rangeverification', ['core'])
    obj.source = [
	'101202-rangeverification.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]

    obj = bld.create_ns3_program('101203-associationtests', ['core'])
    obj.source = [
	'101203-associationtests.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Model.cc',
	'LaneChange.cc',
	]
