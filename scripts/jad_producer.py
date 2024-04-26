#!/usr/bin/env python3
"""JAD Producerクラス

ロボット側から上位側へのブリッジとなるメインモジュール
"""

import json, base64
import signal
from datetime import datetime, timedelta, timezone
from threading import Lock

import rospy
from uoa_poc3_msgs.msg import r_state, r_info, r_navi_result, r_emergency_result, r_navi_command

from producer import Producer
from utils import wrap_namespace


class State:
    """ロボット状態クラス

    ロボット状態を上位に送信する
    """

    def __init__(self, producer):
        """コンストラクタ

        Args:
            producer (Producer): Producerクラスのインスタンス
        """

        self._params = wrap_namespace(rospy.get_param('~'))
        self._producer = producer
        self._prev_ms = datetime.now(timezone.utc)
        self._lock = Lock()

    def state_cb(self, state):
        """ロボット状態のコールバック関数

        ロボット状態を上位に送信する処理

        Args:
            state (r_state): ロボット側から受信したデータ
        """

        #rospy.loginfo('subscribe a state message, %s', state)
        now = datetime.now(timezone.utc)
        if now >= self._prev_ms + timedelta(milliseconds=self._params.thresholds.send_delta_ms) and self._lock.acquire(False):
            self._prev_ms = now
            message = {}
            message = {
                'time': state.time,
                'mode': state.mode,
                'robot_id': state.id,
                'errors': [err for err in state.errors if isinstance(err, str) and len(err) > 0],
                'pose': {
                    'point': {
                        'x': state.pose.point.x,
                        'y': state.pose.point.y,
                        'z': state.pose.point.z,
                    },
                    'angle': {
                        'roll': state.pose.angle.roll,
                        'pitch': state.pose.angle.pitch,
                        'yaw': state.pose.angle.yaw,
                    },
                },
                'destination': {
                    'point': {
                        'x': state.destination.point.x,
                        'y': state.destination.point.y,
                        'z': state.destination.point.z,
                    },
                    'angle': {
                        'roll': state.destination.angle_optional.angle.roll,
                        'pitch': state.destination.angle_optional.angle.pitch,
                        'yaw': state.destination.angle_optional.angle.yaw,
                    } if state.destination.angle_optional.valid else None,
                },
                'accuracy': {
                    'covariance': list(state.covariance),
                },
                'battery': {
                    'voltage': state.battery.voltage,
                    'current': state.battery.current_optional.current if state.battery.current_optional.valid else None,
                },
                'metadata': {},
            }
            print("state")
            print(message)
            self._producer.send(json.dumps({
                "data": message
            }), 'data/robot/StateNotify')
            #self._producer.send(json.dumps({
            #    'attrs': message,
            #}), 'data/robot/StateNotify')
            self._lock.release()


class Info:
    """ロボット情報クラス

    ロボット情報を上位に送信する
    """

    def __init__(self, producer):
        """コンストラクタ

        Args:
            producer (Producer): Producerクラスのインスタンス
        """

        self._producer = producer

    def info_cb(self, info):
        """ロボット情報のコールバック関数

        ロボット情報を上位に送信する処理

        Args:
            info (r_info): ロボット側から受信したデータ
        """

        rospy.loginfo('subscribe a info message, %s', info)
        message = {}
        message = {
            'time': info.time,
            'robotSize': {
                'robot_radius': info.robot_size.robot_radius,
                'inflation_radius': info.robot_size.inflation_radius,
                'footprint': [{'x': c.x, 'y': c.y} for c in info.robot_size.footprint],
            },
            'metadata': {},
        }
        print("info")
        print(message)
        self._producer.send(json.dumps({
            "data": message
        }), 'data/robot/LocalPathNotify')
        #self._producer.send(json.dumps({
        #    'attrs': message
        #}), 'data/robot/LocalPathNotify')


class NaviResult:
    """経路コマンド結果クラス

    経路コマンド結果を上位に送信する
    """
    
    def __init__(self, producer):
        """コンストラクタ

        Args:
            producer (Producer): Producerクラスのインスタンス
        """

        self._params = wrap_namespace(rospy.get_param('~'))
        self._producer = producer

    def navi_result_cb(self, result):
        """経路コマンドのコールバック関数

        経路コマンド結果を上位に送信する処理

        Args:
            result (r_navi_result): ロボット側から受信したデータ
        """

        #result.received_costmap.cost_value = list()  # return empty list as 'cost_value' because it's length is too long
        #rospy.loginfo('subscribe a navi result, %s', result)
        #print(result)
        #return
        message = {}
        #message = {'RTC.TimedString': {"tm": {"sec": 0, "nsec": 0},"data": ""}}
        #message["data"] = {
        message = {
        #message[self._params.rb.navi_cmd_name] = {
            self._params.rb.navi_cmd_name: {
                'time': result.time,
                'received_time': result.received_time,
                'received_command': result.received_cmd,
                'received_revision': result.received_revision,
                'received_destination': {
                    'point': {
                        'x': result.received_destination.point.x,
                        'y': result.received_destination.point.y,
                        'z': result.received_destination.point.z,
                    },
                    'angle': {
                        'roll': result.received_destination.angle_optional.angle.roll,
                        'pitch': result.received_destination.angle_optional.angle.pitch,
                        'yaw': result.received_destination.angle_optional.angle.yaw,
                    } if result.received_destination.angle_optional.valid else None,
                },
                'received_costmap': {
                    'resolution': result.received_costmap.resolution,
                    'width': result.received_costmap.width,
                    'height': result.received_costmap.height,
                    'origin': {
                        'point': {
                            'x': result.received_costmap.origin.point.x,
                            'y': result.received_costmap.origin.point.y,
                            'z': result.received_costmap.origin.point.z,
                        },
                        'angle': {
                            'roll': result.received_costmap.origin.angle.roll,
                            'pitch': result.received_costmap.origin.angle.pitch,
                            'yaw': result.received_costmap.origin.angle.yaw,
                        },
                    },
                    'cost_value': base64.b64encode(result.received_costmap.cost_value).decode(),
                    # 'cost_value': base64.b64encode(result.received_costmap.cost_value),
                    # 'cost_value': result.received_costmap.cost_value,
                },
                'result': result.result,
                'errors': [err for err in result.errors if isinstance(err, str) and len(err) > 0],
            }
        }
        print("naviresult")
        print(message)
        self._producer.send(json.dumps({
            "data": message
        }), 'cmd/robot/NaviResponse')
        #self._producer.send(json.dumps({
        #    'cmdexe': message
        #}), 'cmd/planner/NaviResult')


class EmgResult:
    """緊急コマンド結果クラス

    緊急コマンドを上位に送信する
    """

    def __init__(self, producer):
        """コンストラクタ

        Args:
            producer (Producer): Producerクラスのインスタンス
        """

        self._params = wrap_namespace(rospy.get_param('~'))
        self._producer = producer

    def emg_result_cb(self, result):
        """緊急コマンドのコールバック関数

        緊急コマンド結果を上位側に送信する処理

        Args:
            result (r_emergency_result): ロボット側から受信したデータ
        """

        #rospy.loginfo('subscribe a emg result, %s', result)
        #return
        message = {}
        message[self._params.rb.emg_cmd_name] = {
            'time': result.time,
            'received_time': result.received_time,
            'received_stopCommand': result.received_emergency_cmd,
            'result': result.result,
            'errors': [err for err in result.errors if isinstance(err, str) and len(err) > 0],
        }
        self._producer.send(json.dumps({
            "data": message
        }), 'cmd/robot/EmergencyResponse')
        #self._producer.send(json.dumps({
        #    'cmdexe': message
        #}), 'cmd/robot/EmergencyResponse')


def main():
    """メイン処理

    ロボット側のrospyメッセージを受信し、上位側にメッセージを送信する処理
    """

    rospy.init_node('amqp_attr', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    producer = Producer()

    state = State(producer)
    rospy.Subscriber(params.topic.state, r_state, state.state_cb)

    info = Info(producer)
    rospy.Subscriber(params.topic.info, r_info, info.info_cb)

    navi_result = NaviResult(producer)
    rospy.Subscriber(params.topic.navi_cmdexe, r_navi_result, navi_result.navi_result_cb)
    #rospy.Subscriber("navi_cmd", r_navi_command, navi_result.navi_result_cb)

    emg_result = EmgResult(producer)
    rospy.Subscriber(params.topic.emg_cmdexe, r_emergency_result, emg_result.emg_result_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    producer.run(producer._connect)
    producer.loop_forever()

    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
