#!/usr/bin/env python
import os
import rospy
import threading

from flask import Flask
from flask_ask import Ask, question, statement
# from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


app = Flask(__name__)
ask = Ask(app, "/")

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('naviIntent', PoseStamped, queue_size=10)
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
    msg = PoseStamped()
    msg.header.seq = 1
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    
    msg.pose.position.x = 1.0
    msg.pose.position.y = 1.0
    msg.pose.position.z = 0.0
   
    # quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.radians(wp[0].transform.rotation.yaw))
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0

    pub.publish(msg)
    
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
    
