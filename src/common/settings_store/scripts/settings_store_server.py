#!/usr/bin/env python
# coding: utf-8

# import settings_store.srv
import settings_store.srv
import settings_store.msg
import rospy
import sys
import codecs
import json
import os

sSettings = {}
sRosPublisher = None

def loadSettings(pFileName):
	if pFileName is None:
		return {}
	if not os.path.isfile(pFileName):
		print('store_settings create file ' + pFileName)
		saveSettings(pFileName,{});
	with codecs.open(pFileName,'r',encoding='utf8') as f:
		lSettings = json.load(f)
		if lSettings is None:
			lSettings = {}
		# clear inuse flag
		for k in lSettings:
			lSettings[k]['inuse'] = False
		return lSettings
	raise Exception('Fail to load settings')
def saveSettings(pFileName,pSettings):
	if pFileName is None:
		return
	with codecs.open(pFileName,'w+',encoding='utf8') as f:
		json.dump(pSettings,f, sort_keys=True,indent=4)


def emitChange(pField):
	lName = ''
	lType = ''
	lValue = ''
	lInUse = False

	if pField in sSettings:
		lName = pField
		lType = sSettings[pField]['type']
		lValue = sSettings[pField]['value']
		lInUse = sSettings[pField]['inuse']
	sRosPublisher.publish(settings_store.msg.change(lName,lType,lValue,lInUse))

def handle_declareandget(req):
	lNeedToWrite = False
	if not req.name in sSettings:
		print('settings_store add new entry ' + req.name + '/' + req.type)
		lNeedToWrite = True
	elif sSettings[req.name]['type'] != req.type:
		print('settings_store type missmatch ' + req.name + ' got ' + req.type + ' was expecting ' + sSettings[req.name]['type'])
		lNeedToWrite = True
	elif req.overwritewithdefault and sSettings[req.name]['value'] != req.defaultvalue:
		lNeedToWrite = True
		
	if lNeedToWrite:
		sSettings[req.name] = {
			'value':req.defaultvalue,
			'type':req.type,
			'inuse':True
		}
		emitChange(req.name)
	elif not sSettings[req.name]['inuse']:
		sSettings[req.name]['inuse'] = True
		emitChange(req.name)
	return settings_store.srv.declareandgetResponse(sSettings[req.name]['value'])

def handle_multiget(req):
	lNames = []
	lVals = []
	lTypes = []
	lInUses = []
	
	lRequestedNames = req.requestednames
	if len(lRequestedNames) == 0:
		lRequestedNames = sorted(sSettings.keys())

	for lName in lRequestedNames:
		lNames.append(lName)
		if lName in sSettings:
			if req.setinuse and not sSettings[lName]['inuse']:
				sSettings[lName]['inuse'] = True
				emitChange(lName)
			lVals.append(sSettings[lName]['value'])
			lTypes.append(sSettings[lName]['type'])
			lInUses.append(sSettings[lName]['inuse'])
		else:
			lVals.append('')
			lTypes.append('')
			lInUses.append(False)
	
	return settings_store.srv.multigetResponse(lNames,lVals,lTypes,lInUses)
	
def handle_set(req):
	lError = False
	
	if not req.name in sSettings:
		lError = True
		print('settings_store could not set ' + req.name + ' entry missing')
	elif sSettings[req.name]['type'] != req.type :
		lError = True
		print('settings_store could not set ' + req.name + ' type missmatch got ' + req.type + ' expect ' + sSettings[req.name]['type'])
	elif sSettings[req.name]['value'] != req.newvalue:
		sSettings[req.name]['value'] = req.newvalue
		emitChange(req.name)
	return settings_store.srv.setResponse(lError)
	
def handle_delete(req):
	lError = False
	
	if req.name in sSettings and sSettings[req.name]['inuse']:
		lError = True
		print('settings_store could not delete ' + req.name + ' it is used by a node')
	elif req.name in sSettings:
		del sSettings[req.name]
		# advertise that a setting was deleted
		emitChange(None)
	return settings_store.srv.deleteResponse(lError)

if __name__ == "__main__":
	# if len(sys.argv) != 2:
		# raise Exception('Need a file name in parameter')
	# lSettingsFile = sys.argv[1]
	# if len(sys.argv[1]) == 0:
		# print('settings_store will not be save on disk')
	# else:
		# sSettingsFile = sys.argv[1];
		# print('settings_store will be saved on ' + sSettingsFile)
	
	# sSettings = loadSettings(lSettingsFile)
	
	rospy.init_node('settings_store')
	sRosPublisher = rospy.Publisher('settings_store/change', settings_store.msg.change, queue_size=100)
	
	lServiceGet = rospy.Service('settings_store/declareandget', settings_store.srv.declareandget, handle_declareandget)
	lServiceGet = rospy.Service('settings_store/set', settings_store.srv.set, handle_set)
	lServiceGet = rospy.Service('settings_store/multiget', settings_store.srv.multiget, handle_multiget)
	lServiceGet = rospy.Service('settings_store/delete', settings_store.srv.delete, handle_delete)
	
	# advertise that settings were loaded and are ready
	# emitChange(None)
	
	rospy.spin()
	
	# saveSettings(lSettingsFile,sSettings)