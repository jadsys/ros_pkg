import rospy

import ssl

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

class Handler(mqtt.Client):
    def __init__(self, host, port, use_tls, username, password, client_id, clean_session, userdata, protocol, transport, keepalive, bind_address, ca_certs, certfile, keyfile):
        super().__init__(client_id, clean_session, userdata, protocol, transport)
        rospy.loginfo('init Handler')
        self.host = host
        self.port = port
        self.use_tls = use_tls
        self.username = username
        self.password = password
        self.keepalive = keepalive
        self.bind_address = bind_address
        self.ca_certs = ca_certs
        self.certfile = certfile
        self.keyfile = keyfile

    def ssl_alpn(self):
        ssl_context = ssl.create_default_context()
        #ssl_context.set_alpn_protocols([])
        ssl_context.load_verify_locations(self.ca_certs)
        ssl_context.load_cert_chain(self.certfile, self.keyfile)
        print("ssl alpn")
        return ssl_context

    def run(self, callback):
        rospy.loginfo('start Handler')
        # todo: 
        # self.username_pw_Set(self.username, self.password)
        
        self.on_connect = callback
        # todo: tls
        if self.use_tls:
            # self.tls_set()
            ssl_context = self.ssl_alpn()
            self.tls_set_context(ssl_context)

        self.connect(self.host, self.port, self.keepalive)
        rospy.loginfo('connected to %s', self.host)

    def shutdown(self):
        rospy.loginfo('shutting down Handler...')
        #self.loop_stop()
        self.disconnect()
