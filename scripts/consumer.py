import rospy

from handler import Handler
from utils import wrap_namespace


class Consumer(Handler):
    def __init__(self, callback):
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
        rospy.loginfo('init Consumer')
        self.callback = callback

    def _connect(self, client, userdata, flags, rc):
        rospy.loginfo('chackpoint_3')
        rospy.loginfo('start Consumer')
        self.on_message = self._receive_message
        self.subscribe(self._params.mqtt.queue, self._params.mqtt.qos)
        rospy.loginfo(self._params.mqtt.host)
        #self.loop_start()

    def _receive_message(self, client, userdata, message):        
        #rospy.loginfo('receive a message, %s', message)
        msg = message.payload.decode()
        rospy.loginfo('receive a message, %s', msg)
        rospy.loginfo('topic, %s', message.topic)
        try:
            self.callback(msg)
        except Exception as e:
            rospy.logerr('error when calling callback, %s', e)
    

