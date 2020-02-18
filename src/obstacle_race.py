#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------
#Title: 物を運んで障害物を避けながらゴールを目指すプログラム
#Author: Ishiyama Yuki
#Data: 2020/2/13 74 def main():

#MeMo
#--------------------------------------------------------------

import rospy
import sys
import smach
import smach_ros

#sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
#from common_action_client import *
#from common_function import *

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome1','outcome3','outcome5'],
                            input_keys=['location_in'],
                            output_keys=['location_out'])
    

    def naviAC(self, object_name):
        flag = 'failed'
        location_list = searchLocationName(object_name)
        while not rospy.is_shutdown() and self.flag == 'failed':
            self.flag = navigationAC(location_list)
            rospy.sleep(1.0)


    def execute(self, userdata):
        if userdata.location_in == 'cupboard':
            naviAC(userdata.location_in)
            return 'outcome1'
        
        elif userdata.location_in != 'cupboard' and userdata.location_in != 'door':
            naviAC(userdata.location_in)
            userdata.location_out = 'door'
            return 'outcome3'
        
        elif userdata.location_in == 'door':
            naviAC(userdata.location_in)
            return 'outcome5'


class GrabArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome2'],
                            output_keys=['object_name_out'])
        self.grab = rospy.Service('ManipulateSrv', String, execute)
        self.object_name = String

    def execute(self, userdata):
        userdata.object_name_out = 'cup'
        grab(object_name_out)
        print 'outcome2'
        return 'outcome2'


class OpenDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['outcome3','outcome4'],
                            output_keys=['location_out'])

    def execute(self, userdata):
        enterTheRoomAC()
        userdata.location_out = 'goal'
        print 'outcome4'
        return 'outcome4'

        

def main():
    rospy.loginfo('Start "task"')
    sm = smach.StateMachine(outcomes=['outcome6'])
    sm.userdata.sm_target = 'cupboard'
    with sm:
        smach.StateMachine.add('NAV', Navigation(),
                transitions={
                            'outcome1':'ARM',
                            'outcome3':'DOOR',
                            'outcome5':'outcome6'},
                remapping={
                    'location_in':'sm_target',
                    'location_out':'sm_target'})

        smach.StateMachine.add('ARM', GrabArm(),
                transitions={
                    'outcome2':'NAV'},
                remapping={
                    'object_name_out':'sm_target'})

        smach.StateMachine.add('DOOR', OpenDoor(),
                transitions={
                            'outcome3':'DOOR',
                            'outcome4':'NAV'},
                remapping={
                    'location_out':'sm_target'})
    outcome = sm.execute()
    rospy.loginfo('Finish"task"')

if __name__ == '__main__':
    rospy.init_node('obstacle_race')
    main()
