#!/usr/bin/env python3
"""JAD Consumerモジュール

上位側からロボット側へのブリッジとなるメインモジュール
"""

import json
import signal

import rospy
from geometry_msgs.msg import Point
from uoa_poc3_msgs.msg import r_navi_command, r_pose_optional, r_angle_optional, r_angle, r_costmap, r_pose, r_emergency_command, r_navi_result, r_state, r_info, r_corner

from PIL import Image, ImageDraw, ImageOps

from consumer import Consumer
from utils import wrap_namespace

FREE = 0
OBSTACLE = 254


class Dispatcher:
    """Dispacherクラス

    イベント実行する
    """

    def __init__(self, naviCommand, emgCommand):
    #def __init__(self, naviCommand, emgCommand, naviCommandExe, stateTest, infoTest):
        """コンストラクタ

        Args:
            naviCommand (NaviCommand): 経路コマンドのインスタンス
            emgCommand (EmgCommand): 緊急コマンドのインスタンス
        """
        self._params = wrap_namespace(rospy.get_param('~'))
        self._naviCommand = naviCommand
        self._emgCommand = emgCommand
        #self._naviCommandExe = naviCommandExe
        #self._stateTest = stateTest
        #self._infoTest = infoTest


    def dispatch_cb(self, msg):
        """上位側からメッセージ受信時のコールバック関数
        
        受信したメッセージに応じた処理を実行する

        Args:
            msg (dict): 上位からの受信メッセージ
        """
        rospy.loginfo('consume a command message, %s', msg)
        try:
            message2 = json.loads(msg)
            # if "RTC.TimedString" not in message:
            #     print("rtc timedstring")
            #     return
            # message2 = message['RTC.TimedString']['data']
            # message2 = json.loads(message2)

            # if "cmd" not in message2:
            #     rospy.logerr('invalid payload')
            #     return
            # if self._params.rb.navi_cmd_name in message2['cmd']:
            if self._params.rb.navi_cmd_name in message2:
                #self._naviCommandExe.process()
                #self._stateTest.process()
                #self._infoTest.process()
                
                body = message2[self._params.rb.navi_cmd_name]['value']
                if self._params.rb.entity_id != body['robot_id']:
                    return
                ros_published = self._naviCommand.process(body)
                ros_published.costmap.cost_value = []  # clear 'cost_value' because it's length is too long
                rospy.loginfo('processed the navi command, %s', ros_published)
            elif self._params.rb.emg_cmd_name in message2:
                body = message2[self._params.rb.emg_cmd_name]['value']
                ros_published = self._emgCommand.process(body)
                rospy.loginfo('processed the emg command, %s', ros_published)
            else:
                rospy.logerr('unknown command')
        except (ValueError, TypeError) as e:
            rospy.logerr('invalid payload, %s', e)


#producer test
# class NaviCommandExe:
#     """経路コマンド
    
#     経路コマンドをテスト送信するクラス
#     """
    
#     def __init__(self, publisher):
#         """コンストラクタ

#         Args:
#             publisher (rospy.Publisher): Publisherクラスのインスタンス
#         """
#         self._params = wrap_namespace(rospy.get_param('~'))
#         self._publisher = publisher

#     def process(self):
#         #rospy.loginfo('process a navi message %s', body)

#         command = r_navi_result()
#         command.id = "megarover_02"
#         command.type = "megarover"
#         command.time = "2019-06-07 08:39:42"
#         command.received_time = "2019-06-07 08:39:40"
#         command.received_cmd = "navi"
#         command.received_destination.point.x = 0.503
#         command.received_destination.point.y = 0.0
#         command.received_destination.point.z = 0.0
#         command.received_destination.angle_optional.valid = False
#         command.received_destination.angle_optional.angle.roll = 0.0
#         command.received_destination.angle_optional.angle.pitch = 0.0
#         command.received_destination.angle_optional.angle.yaw = 0.0
#         command.received_costmap.resolution = 0.05
#         command.received_costmap.width = 10
#         command.received_costmap.height = 10
#         command.received_costmap.origin.point.x = 1.0
#         command.received_costmap.origin.point.y = 1.0
#         command.received_costmap.origin.point.z = 0.0
#         command.received_costmap.origin.angle.roll = 0.0
#         command.received_costmap.origin.angle.pitch = 0.0
#         command.received_costmap.origin.angle.yaw = 0.0
#         command.received_costmap.cost_value = [0, 1, 0, 1, 0, 1]
#         command.result = "ack"
#         command.errors = []

#         self._publisher.publish(command)
#         return command

# class State:
#     """ロボット状態クラス

#     ロボットの状態を上位に送信する
#     """

#     def __init__(self, publisher):
#         """コンストラクタ
        
#         Args:
#             publisher (rospy.Publisher): Publisherクラスのインスタンス
#         """

#         self._params = wrap_namespace(rospy.get_param('~'))
#         self._publisher = publisher

#     def process(self):
#         """実行処理
        
#         ロボットの状態を上位に送信する処理

#         Returns:
#             r_state: ロボット状態コマンド
#         """
#         #rospy.loginfo('process a navi message %s', body)

#         command = r_state()
#         command.id = "megarover_02"
#         command.type = "megarover"
#         command.time = "2019-06-07 08:39:42"
#         command.mode = "navi"
#         command.errors = []
#         command.pose.point.x = 3.402
#         command.pose.point.y = 1.015
#         command.pose.point.z = -0.002
#         command.pose.angle.roll = 0.0
#         command.pose.angle.pitch = 0.0
#         command.pose.angle.yaw = 0.0
#         command.destination.point.x = 0.01
#         command.destination.point.y = 0.02
#         command.destination.point.z = 0.03
#         command.destination.angle_optional.valid = True
#         command.destination.angle_optional.angle.roll = 1
#         command.destination.angle_optional.angle.pitch = 1
#         command.destination.angle_optional.angle.yaw = 1
#         command.covariance[0] = 0.1
#         command.covariance[1] = 0.0
#         #command.covariance = []
#         command.battery.voltage = 11.0
#         command.battery.current_optional.valid = True
#         command.battery.current_optional.current = 0.23
#         print(command)

#         self._publisher.publish(command)
#         return command

# class Info:
#     """ロボット情報クラス
    
#     ロボット情報をロボット側に送信する
#     """

#     def __init__(self, publisher):
#         """コンストラクタ

#         Args:
#             publisher (rospy.Publisher): Publisherクラスのインスタンス
#         """

#         self._params = wrap_namespace(rospy.get_param('~'))
#         self._publisher = publisher

#     def process(self):
#         """実行処理

#         ロボット情報をロボット側に送信する処理

#         Returns:
#             r_info: ロボット情報コマンド
#         """

#         #rospy.loginfo('process a navi message %s', body)

#         command = r_info()
#         command.id = "megarover_02"
#         command.type = "megarover"
#         command.time = "2019-06-07 08:39:42"
#         command.robot_size.robot_radius = 0.17
#         command.robot_size.inflation_radius = 0.35
#         corner = r_corner()
#         command.robot_size.footprint = []
#         #corner = r_corner()
#         #corner.x = -0.25
#         #corner.y = -0.15
#         #command.robot_size.footprint[0] = corner
#         #corner.x = -0.25
#         #corner.y = 0.15
#         #command.robot_size.footprint[1] = corner
#         #corner.x = 0.07
#         #corner.y = 0.15
#         #command.robot_size.footprint[2] = corner
#         #corner.x = 0.07
#         #corner.y = -0.15
#         #command.robot_size.footprint[3] = corner
#         print(command)

#         self._publisher.publish(command)
#         return command



class NaviCommand:
    """経路コマンドクラス

    経路コマンドをロボット側に送信する
    """

    def __init__(self, publisher):
        """コンストラクタ

        Args:
            publisher (rospy.Publisher): Publisherクラスのインスタンス
        """

        self._params = wrap_namespace(rospy.get_param('~'))
        self._publisher = publisher

    def process(self, body):
        """実行処理

        経路コマンドをロボット側に送信する処理

        Args:
            body (dict): 送信するデータ

        Returns:
            r_navi_command: 経路コマンド
        """

        rospy.loginfo('process a navi message %s', body)

        command = r_navi_command()
        command.id = self._params.rb.entity_id
        command.type = self._params.rb.entity_type
        command.time = body['time']
        command.cmd = body['command']
        command.revision = body['revision']

        angle = r_angle()
        angle_optional = r_angle_optional()
        if not body['destination']['angle']:
            angle.roll = 0.0
            angle.pitch = 0.0
            angle.yaw = 0.0
            angle_optional.valid = False
            angle_optional.angle = angle
        else:
            angle.roll = body['destination']['angle']['roll']
            angle.pitch = body['destination']['angle']['pitch']
            angle.yaw = body['destination']['angle']['yaw']
            angle_optional.valid = True
            angle_optional.angle = angle
        point = Point()
        point.x = body['destination']['point']['x']
        point.y = body['destination']['point']['y']
        point.z = body['destination']['point']['z']
        destination = r_pose_optional()
        destination.point = point
        destination.angle_optional = angle_optional
        command.destination = destination

        costmap = r_costmap()
        costmap.resolution = body['metadata']['costmap']['resolution']
        costmap.width = body['metadata']['costmap']['width']
        costmap.height = body['metadata']['costmap']['height']
        opoint = Point()
        opoint.x = body['metadata']['costmap']['origin']['point']['x']
        opoint.y = body['metadata']['costmap']['origin']['point']['y']
        opoint.z = body['metadata']['costmap']['origin']['point']['z']
        oangle = r_angle()
        oangle.roll = body['metadata']['costmap']['origin']['angle']['roll']
        oangle.pitch = body['metadata']['costmap']['origin']['angle']['pitch']
        oangle.yaw = body['metadata']['costmap']['origin']['angle']['yaw']
        origin = r_pose()
        origin.point = opoint
        origin.angle = oangle
        costmap.origin = origin
        costmap.cost_value = self._calc_cost(body['waypoints'], body['metadata']['inflation_radius'], costmap)
        command.costmap = costmap

        self._publisher.publish(command)
        return command

    def _calc_cost(self, waypoints, radius, costmap):
        """コスト計算処理

        waypointからコストマップを生成する処理

        Args:
            waypoints (dict): 経路
            radius (float): ロボットの半径
            costmap (r_costmap): コストマップ

        Returns:
            list: コストマップの画像データ
        """

        img = Image.new('L', (costmap.width, costmap.height), color=OBSTACLE)
        draw = ImageDraw.Draw(img)
        nodes = [(int(n['point']['x']), int(n['point']['y'])) for n in waypoints]
        r = int(radius/costmap.resolution)
        for node in nodes:
            draw.ellipse((node[0] - r, node[1] - r, node[0] + r, node[1] + r), fill=FREE)
        draw.line(nodes, fill=FREE, width=int(r * 2))
        flip = ImageOps.flip(img)
        return list(flip.getdata())


class EmgCommand:
    """緊急コマンドクラス

    緊急コマンドをロボット側に送信する
    """

    def __init__(self, publisher):
        """コンストラクタ

        Args:
            publisher (rospy.Publisher): Publisherクラスのインスタンス
        """

        self._params = wrap_namespace(rospy.get_param('~'))
        self._publisher = publisher

    def process(self, body):
        """実行処理

        緊急コマンドをロボット側に送信する処理
        
        Args:
            body (dict): 送信するデータ

        Returns:
            r_emergency_command: 緊急コマンド
        """

        rospy.loginfo('process an emg message %s', body)

        command = r_emergency_command()
        command.id = self._params.rb.entity_id
        command.type = self._params.rb.entity_type
        command.time = body['time']
        command.emergency_cmd = body['stopCommand']

        self._publisher.publish(command)
        return command


def main():
    """メイン処理

    上位側のMQTTメッセージを受信し、ロボット側にメッセージを送信する処理
    """

    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))
    rospy.loginfo('chackpoint_1')

    #producer test
    #naviexe_pub = rospy.Publisher(params.topic.navi_cmdexe, r_navi_result, queue_size=1)
    #state_pub = rospy.Publisher(params.topic.state, r_state, queue_size=1)
    #info_pub = rospy.Publisher(params.topic.info, r_info, queue_size=1)
    
    navi_pub = rospy.Publisher(params.topic.navi_cmd, r_navi_command, queue_size=1)
    emg_pub = rospy.Publisher(params.topic.emg_cmd, r_emergency_command, queue_size=1)

    #producer test
    #naviCommandexe = NaviCommandExe(naviexe_pub)
    #stateTest = State(state_pub)
    #infoTest = Info(info_pub)
    
    naviCommand = NaviCommand(navi_pub)
    emgCommand = EmgCommand(emg_pub)
    #dispatcher = Dispatcher(naviCommand, emgCommand, naviCommandexe, stateTest, infoTest)
    dispatcher = Dispatcher(naviCommand, emgCommand)
    consumer = Consumer(dispatcher.dispatch_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    consumer.run(consumer._connect)
    rospy.loginfo('chackpoint_2')
    
    consumer.loop_forever()

    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
