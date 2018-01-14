#!/usr/bin/env python
# coding: utf-8

import template_python.srv
import template_python.msg
import rospy
import sys
import codecs
import json
import os

sRosPublisher = None


def emitMsg(pVal):
	sRosPublisher.publish(template_python.msg.msg(pVal))

def handle_service(req):
	emitMsg(req.name)
	return template_python.srv.serviceResponse(req.in)

if __name__ == "__main__":
	rospy.init_node('template_python')
	sRosPublisher = rospy.Publisher('template_python/msg', template_python.msg.msg, queue_size=100)
	
	lServiceGet = rospy.Service('template_python/service', template_python.srv.service, handle_service)
	emitMsg('Start')
	
	rospy.spin()
	