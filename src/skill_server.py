#!/usr/bin/env python
import os
import rospy
import threading

from flask import Flask
from flask_ask import Ask, question, statement
from std_msgs.msg import Int32
app = Flask(__name__)
ask = Ask(app, "/")

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('i2r_util_manual_mission/manual_trigger', Int32, queue_size=1)
NGROK = rospy.get_param('/ngrok', None)


@ask.launch
def launch():
    '''
    Executed when launching skill.
    To invoke skill: say "Alexa, open kah toe"
    '''
    welcome_sentence = 'Hello, this is kah toe, where do you want me to go?'
    return question(welcome_sentence)


@ask.intent('NavigationIntent', default={'location':""})
def navi_intent_function(location):
    '''
    Executed when NavigationIntent is called:
    To invoke navigation intent say "go to {location} / move to {location} /
    bring me to {location}"
    '''
    msg = Int32()
    if location == "main door":
        msg.data = 1 
    elif location == "roti place":
        msg.data = 2
    elif location == "rojak place":
        msg.data = 3
    elif location == "mee siam place":
        msg.data = 4
    elif location == "laksa place":
        msg.data = 5
    elif location == "big place":
        msg.data = 6
    elif location == "water cooler":
        msg.data = 7
    elif location == "the little hut":
        msg.data = 8
    elif location == "mail room":
        msg.data = 9
    elif location == "patrol":
        msg.data = 10
    pub.publish(msg)
    print(location)
    
    return statement('Ok, I am moving to {}.'.format(location))


@ask.session_ended
def session_ended():
    return "{}", 200


if __name__ == '__main__':
    if NGROK:
        print 'NGROK mode'
        app.run(host=os.environ['ROS_IP'], port=1234)
    else:
        print 'Manual tunneling mode'
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
    
