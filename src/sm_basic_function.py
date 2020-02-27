#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
#Title: 物を運んで障害物を避けながらゴールを目指すプログラム
#Author: Ishiyama Yuki
#Data: 2020/2.20 
#Memo
#-------------------------------------------------------------------
import time
import sys

import rospy
import smach
import smach_ros

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *


class EnterRoom(smac.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_pap'])
    
    def execute(self):
        rospy.loginfo('Enter The Room')
        #enterTheRoomAC()
        return 'to_pap'


class MoveAndPick(smach.State):
    def __init__(self, userdata):
        smach.State.__init__(self,
                            outcomes=['success', 'failed'],
                            output_keys=['oubject_name_out'])
        #Service
        self.grab = rospy.Service('ManipulateSrv', String, execute)

       #parameter 
        self.location_name = 'table'
        self.flag = 'failed'

    def execute(self):
        speak('start pick and place')
        location_list = searchLocationName(self.location_name)
        while not rospy.is_shutdown() and self.flag == 'failed':
            self.flag = navigationAC(location_list)
            rospy.sleep(1.0)
        userdata.object_name_out = 'cup'
        self.grab(userdata.object_name_out)  #object_nameによってif等で条件分岐
       # if :
        return 'success'
       # elif:
       # return 'failed'

class MoveAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['object_name_in'])
        self.flag = 'failed'
        self.object_list = ['cup','bottle','snack','dish','chips',
                            'bag','toy','smartphone','book','pen']

    def execute(self):
        if userdata.object_name_in  in self.object_list:
            while  not rospy.is_shutdown() and self.flag == 'failed':
                self.flag = navigationAC('chair')
                rospy.sleep(1.0)
        else:
            while  not rospy.is_shutdown() and self.flag == 'failed':
                self.flag = navigationAC('trash box')
                rospy.sleep(1.0)
        #ここにplace関数を挿入する
        return 'completed'


class AvoidThat(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_WDYS'])
        self.flag = 'failed'
        target_point = 'operator'

    def execute(self):
        speak('start Avoid That')
        while not rospy.is_shutdown() and self.flag == 'failed':
            self.flag = navigationAC(target_point)
            rospy.sleep(1.0)
        return 'to_WDYS'


class TimeCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_PersonSearch']
                            output_keys=['start_time_out'])
    def execute(self):
        speak('Staet What did you say')
        start_time_out = time.time()
        return 'to_PersonSearch'


class PersonSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['found'])
        self.flag = 'failed'

    def execute(self):
        #人発見プログラムを起動
        #m6Control(0.5)
        #speak('found Person')
        return 'found'


class QuestionResponse(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['continues', 'give_up', 'completed'],
                            input_keys=['success_count_input' , 'start_time_in'],
                            output_keys=['success_count_output'])
       # WhatDidYouSay.srv()
       target_time = 150

    def execute(self, userdata):
        end_time = time.time()
        if end_time - userdata.start_time_in >= target_time:
            speak('Cancel Q and A session')
            return 'give_up'
        #resalt = WhatDidYouSay.srv()の呼び出し
        if resalt == True:
            success_count_input + 1 = success_count_output
            if success_count_output == 3:
                return 'completed'
            elif success_count_output != 3:
                return 'continues'


class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['to_finish'])
        self.flag = 'failed'

    def execute(slef):
        speak('Go to the exit')
        while not rospy.is_shutdown and self.flag == 'failed':
            navigationAC('Exit')
            rospy.sleep(1.0)
        return 'to_finish'


def main():
    sm_top = smach.StateMachine(outcomes=['FINISH'])

    with sm_top:
        ### EnterRoom
        smch.StateMachine.add('ENTER',EnterRoom(),
                            trasitions={'to_pap':'sm_pap'})

        ### Pick and place
        sm_pap = smach.StateMachine(outcomes=['to_AvoidThat'])
        with sm_pap:
            smach.StateMachine.add('pick', MoveAndPick(),
                            trasitions={'success':'place',
                                        'failed':'to_AvoidThat'})
            smach.StateMachine.add('place', MoveAndPlace(),
                            trasitions={'completed':'to_AvoidThat'})
        smach.StateMachine.add('PICH_AND_PLACE', sm_pap,
                            trasitions={'to_AvoidThat':'AVOID_THAT'})

        ### Avoid that
        smach.StateMachine.add('AVOIDTHAT', AvoidThat(),
                            trasitions={'to_WDYS':'WHAT_DID_YOU_SAY'})

        ### what did you say
        sm_wdys = smach.StateMachine(outcomes=['give_up','completed'])
        sm.userdata.sm_time = 0
        with sm_wdys:
            smach.StateMachine.add('STARTWDYS':TimeCount(),
                            trasitions={'to_PS':'PersonSearch'},
                            remapping={'start_time_out':'sm_time'})
            smach.StateMachine.add('PersonSearch', PersonSearch(),
                            trasitions={'found':'QUESTION'})
            smach.StateMachine.add('QUESTION', QuestionResponse(),
                    trasitions={'continues':'QUESTION',
                                'give_up':'to_exit',
                                'completed':'to_exit'},
                    remapping={'success_count_input':'sm_success',
                               'success_count_output':'sm_success',
                               'start_time__in':'sm_time'})
        smach.StateMachine.add('WHAT_DID_YOU_SAY', sm_wdys,
                    trasitions={'to_exit':'EXIT')}

        ### Go to the exit
        smach.StateMachine.add('EXIT', ExitRoom(),
                    trasitions{'to_finish':'FINISH'})

        outcome = sm.execute()
        
if __name__ == '__main__':
    rospy.init_node('sm_basic_function')
    main()
