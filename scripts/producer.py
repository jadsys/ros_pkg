import rospy


from handler import Handler
from utils import wrap_namespace


class Producer(Handler):
    def __init__(self):
        self._params = wrap_namespace(rospy.get_param('~'))
        super().__init__(self._params.mqtt.host,
                         self._params.mqtt.port,
                         self._params.mqtt.use_tls,
                         self._params.mqtt.username,
                         self._params.mqtt.password,
                         self._params.mqtt.client_id,
                         self._params.mqtt.clean_session,
                         self._params.mqtt.user_data,
                         self._params.mqtt.protocol,
                         self._params.mqtt.transport,
                         self._params.mqtt.keepalive,
                         self._params.mqtt.bind_address,
                         self._params.mqtt.tls.ca_certs,
                         self._params.mqtt.tls.certfile,
                         self._params.mqtt.tls.keyfile
                         )
        rospy.loginfo('init Producer')
        self.is_sendable = False

    def _connect(self, client, serdata, flags, rc):
        rospy.loginfo('start Producer')
        self.is_sendable = True
        #self.loop_start()

    def send(self, msg, topic):
        if self.is_sendable:
            #if topic is None:
            #    topic = self._params.mqtt.queue
            #topic = 'robot_project/jad4lict23k/test'
            rospy.loginfo('send a message, %s', msg)
            print(topic)
            #print(msg)
            self.publish(topic, msg)
            #self.publish(self._params.mqtt.queue, msg)
