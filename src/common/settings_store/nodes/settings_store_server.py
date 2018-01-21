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
		rospy.loginfo('store_settings create file ' + pFileName)
		saveSettings(pFileName,{});
	with codecs.open(pFileName,'r',encoding='utf8') as f:
		lSettings = json.load(f)
		if lSettings is None:
			lSettings = {}
		# clear inuse flag
		for k in lSettings:
			lSettings[k]['inuse'] = False
		return lSettings
	rospy.logerr('Fail to load settings')
	raise Exception()
def saveSettings(pFileName,pSettings):
	if pFileName is None:
		return
	with codecs.open(pFileName,'w+',encoding='utf8') as f:
		json.dump(pSettings,f, sort_keys=True,indent=4)


def emitChange(pField):
	lName = ''
	lValue = ''
	lInUse = False

	if pField in sSettings:
		lName = pField
		lValue = sSettings[pField]['value']
		lInUse = sSettings[pField]['inuse']
	sRosPublisher.publish(settings_store.msg.change(lName,lValue,lInUse))

def handle_declareandget(req):
	if len(req.names) != len(req.defaultvalues):
		rospy.logfatal('names and defaultvalues should have same length')
		return None
	lVals = []
	lChanges = []
	for (name, defaultval) in zip(req.names, req.defaultvalues):
		if not name in sSettings:
			sSettings[name] = {'value':defaultval,'inuse':True}
			lChanges.append(name)
		elif not sSettings[name]['inuse']:
			sSettings[name]['inuse'] = True
			lChanges.append
		lVals.append(str(sSettings[name]['value']))

	for name in lChanges:
		emitChange(name)

	return settings_store.srv.declareandgetResponse(lVals)

def handle_multiget(req):
	lNames = []
	lVals = []
	lInUses = []
	
	lRequestedNames = req.requestednames
	if len(lRequestedNames) == 0:
		lRequestedNames = sorted(sSettings.keys())

	for lName in lRequestedNames:
		lNames.append(lName)
		if lName in sSettings:
			lVals.append(sSettings[lName]['value'])
			lInUses.append(sSettings[lName]['inuse'])
		else:
			lVals.append('')
			lInUses.append(False)
	
	return settings_store.srv.multigetResponse(lNames,lVals,lInUses)
	
def handle_set(req):
	lError = False
	
	if not req.name in sSettings:
		lError = True
		rospy.logwarn('settings_store could not set ''%'' entry missing' % req.name)
	elif sSettings[req.name]['value'] != req.newvalue:
		sSettings[req.name]['value'] = req.newvalue
		emitChange(req.name)
	return settings_store.srv.setResponse(lError)
	
def handle_delete(req):
	lError = False
	
	if req.name in sSettings and sSettings[req.name]['inuse']:
		lError = True
		rospy.logwarn('settings_store could not delete ''%'' it is used by a node' % req.name)
	elif req.name in sSettings:
		del sSettings[req.name]
		# advertise that a setting was deleted
		emitChange(None)
	return settings_store.srv.deleteResponse(lError)

if __name__ == "__main__":
	import os
	os.getcwd()
	rospy.init_node('settings_store')
	
	lSettingsFile = rospy.get_param('/settings_store/settingsfilename')
	if lSettingsFile is None:
		rospy.logfatal('Need /settings_store/settingsfilename parameter, to disable persistent storage give an empty filename')
		raise Exception()
	if len(lSettingsFile) == 0:
		rospy.logwarn('settings_store will not be save on disk')
	else:
		rospy.loginfo('settings_store will be saved on ' + lSettingsFile)
	sSettings = loadSettings(lSettingsFile)
	
	sRosPublisher = rospy.Publisher('settings_store/Change', settings_store.msg.change, queue_size=100)
	
	lServiceGet = rospy.Service('settings_store/set', settings_store.srv.set, handle_set)
	lServiceGet = rospy.Service('settings_store/multiget', settings_store.srv.multiget, handle_multiget)
	lServiceGet = rospy.Service('settings_store/delete', settings_store.srv.delete, handle_delete)
	# declare this one last, so nodes that wait for settings_store will block until this node is fully initialized
	lServiceGet = rospy.Service('settings_store/declareandget', settings_store.srv.declareandget, handle_declareandget)
	
	# advertise that settings were loaded and are ready
	emitChange(None)
	
	rospy.spin()
	
	saveSettings(lSettingsFile,sSettings)