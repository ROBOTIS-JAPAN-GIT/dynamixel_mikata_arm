#!/usr/bin/env python
# -*- coding: utf-8 -*-
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32, String
from sensor_msgs.msg import JointState
import rospy
import math

motion_player_status=""
joint=[0.0, 0.0, 0.0, 0.0, 0.0]
def joint_state_callback(data):
    global joint
    joint[0] = data.position[0] / math.pi * 180
    joint[1] = data.position[1] / math.pi * 180
    joint[2] = data.position[2] / math.pi * 180
    joint[3] = data.position[3] / math.pi * 180
    joint[4] = data.position[4] / math.pi * 180

def motion_player_status_callback(data):
    global motion_player_status
    motion_player_status = data.data

rospy.init_node("rviz_overlay_text")
rospy.Subscriber("/dxl/joint_state", JointState, joint_state_callback)
rospy.Subscriber("/motion_player/status", String, motion_player_status_callback)

text_pub = rospy.Publisher("rviz_overlay_text", OverlayText, queue_size=1)
counter = 0
rate = 10
r = rospy.Rate(rate)
import random, math
while not rospy.is_shutdown():
  counter = counter + 1
  text = OverlayText()
  theta = counter % 255 / 255.0
  text.width = 560
  text.height = 600
  text.left = 10
  text.top = 10
  text.text_size = 18
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  text.text = """<pre>
  ROBOTIS Mikata Arm ４DOF　デモ

  -----
  自動再生デモ実行状態: %s

  自動再生デモの操作はサービスの呼び出しで行います。
    [/motion_player/start]    : 開始
    [/motion_player/pause]    : 一時停止
    [/motion_player/continue] : 再開
    [/motion_player/stop]     : 終了

  開始前に最下部のモーターの配線が巻き付いていない
  ことを確認して下さい。

  [終了]時はモーターのトルクが即座にオフになります。
  アームの腕先が落下しないようご注意下さい。
  （一時停止の後、腕先を手で支えて終了する等）

  -----
  関節角度 (Degree)
      JOINT1 : %6.1f
      JOINT2 : %6.1f
      JOINT3 : %6.1f
      JOINT4 : %6.1f
      GRIPPER: %6.1f
</pre>
  """ % (motion_player_status, joint[0], joint[1], joint[2], joint[3], joint[4])
  text.fg_color = ColorRGBA(240.0 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  text_pub.publish(text)
#  value_pub.publish(math.sin(counter * math.pi * 2 / 100))
  r.sleep()

