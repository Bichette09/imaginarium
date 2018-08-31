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
sStates = {}
sRosPublisher = None

def loadSettings(pFileName):
	if pFileName is None:
		return {}
	try:
		if not os.path.isfile(pFileName):
			rospy.loginfo('store_settings create file ' + pFileName)
			saveSettings(pFileName,{});
	except:
		rospy.logerr('Fail to create settings file %s' % (pFileName))
		raise Exception()
	
	try:
		with codecs.open(pFileName,'r',encoding='utf8') as f:
			lSettings = json.load(f)
			if lSettings is None:
				lSettings = {}
			
			# clear inuse flag and ensure that description is set
			for k in lSettings:
				if not 'description' in lSettings[k]:
					lSettings[k]['description'] = ''
				lSettings[k]['inuse'] = False
			
			return lSettings
	except:
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
	lDesc = ''
	lInUse = False

	if pField in sSettings:
		lName = pField
		lValue = sSettings[pField]['value']
		lInUse = sSettings[pField]['inuse']
		lDesc = sSettings[pField]['description']
	sRosPublisher.publish(settings_store.msg.Change(lName,lValue,lDesc,lInUse))

def handle_declareandget(req):
	if len(req.names) != len(req.defaultvalues):
		rospy.logfatal('names and defaultvalues should have same length')
		return None
	lVals = []
	lChanges = []
	for (name, defaultval,description) in zip(req.names, req.defaultvalues,req.descriptions):
		if not name in sSettings:
			sSettings[name] = {'value':defaultval,'inuse':True,'description':description}
			lChanges.append(name)
		elif not sSettings[name]['inuse']:
			sSettings[name]['inuse'] = True
			sSettings[name]['description'] = description
			lChanges.append(name)
		lVals.append(str(sSettings[name]['value']))

	for name in lChanges:
		emitChange(name)

	return settings_store.srv.declareandgetResponse(lVals)

def handle_multiget(req):
	lNames = []
	lVals = []
	lDescriptions = []
	lInUses = []
	
	lRequestedNames = req.requestednames
	if len(lRequestedNames) == 0:
		lRequestedNames = sorted(sSettings.keys())

	for lName in lRequestedNames:
		lNames.append(lName)
		if lName in sSettings:
			lVals.append(sSettings[lName]['value'])
			lDescriptions.append(sSettings[lName]['description'])
			lInUses.append(sSettings[lName]['inuse'])
		else:
			lVals.append('')
			lDescriptions.append('')
			lInUses.append(False)
	
	return settings_store.srv.multigetResponse(lNames,lVals,lDescriptions,lInUses)
	
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

def on_state_change(pParam):
	sStates[pParam.name] = pParam.value
	
def handle_get_states(req):
	lNames = []
	lVals = []
	
	lRequestedNames = req.requestednames
	if len(lRequestedNames) == 0:
		lRequestedNames = sorted(sStates.keys())

	for lName in lRequestedNames:
		lNames.append(lName)
		if lName in sStates:
			lVals.append(sStates[lName])
		else:
			lVals.append('')
			
	return settings_store.srv.getstatesResponse(lNames,lVals)
	
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
	
	sRosPublisher = rospy.Publisher('settings_store/Change', settings_store.msg.Change, queue_size=1000)
	
	sStateChangeSubscriber = rospy.Subscriber('settings_store/StateChange',settings_store.msg.Change,on_state_change)

	lServiceGet = rospy.Service('settings_store/getstates', settings_store.srv.getstates, handle_get_states)
	lServiceGet = rospy.Service('settings_store/set', settings_store.srv.set, handle_set)
	lServiceGet = rospy.Service('settings_store/multiget', settings_store.srv.multiget, handle_multiget)
	lServiceGet = rospy.Service('settings_store/delete', settings_store.srv.delete, handle_delete)
	# declare this one last, so nodes that wait for settings_store will block until this node is fully initialized
	lServiceGet = rospy.Service('settings_store/declareandget', settings_store.srv.declareandget, handle_declareandget)
	
	# advertise that settings were loaded and are ready
	emitChange(None)
	
	rospy.spin()
	
	saveSettings(lSettingsFile,sSettings)