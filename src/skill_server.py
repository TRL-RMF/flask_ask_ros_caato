#!/usr/bin/env python
import os
import rospy
import threading

from flask import Flask
from flask_ask import Ask, question, statement
from std_msgs.msg import String

app = Flask(__name__)
ask = Ask(app, "/")

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('naviIntent', String, queue_size=1)
NGROK = rospy.get_param('/ngrok', None)


@ask.launch
def launch():
    '''
    Executed when launching skill.
    To invoke skill: say "Alexa, open kah toe"
    '''
    welcome_sentence = 'Hello, this is kah toe, your personal robotics assistant.'
    return question(welcome_sentence)


@ask.intent('NavigationIntent', default={'location':""})
def navi_intent_function(location):
    '''
    Executed when NavigationIntent is called:
    To invoke navigation intent say "go to {location} / move to {location} / bring me to {location}"
    '''
    pub.publish("x:1.0 y:1.0")
    return statement('Ok, I am on the way to the {}.'.format(location))


@ask.session_ended
def session_ended():
    return "{}", 200


if __name__ == '__main__':
    if NGROK:
        ip='10.10.7.220'
        print 'NGROK mode'
        app.run(host=ip, port=5000)
    else:
        print 'Manual tunneling mode'
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
    
