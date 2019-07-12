#!/usr/bin/python
# -*- coding: utf-8 -*-

import wx
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool

BT_EMERGENCY_RESET = 1
BT_STEER_DRIVE_AUTO = 2
BT_STEER_AUTO = 3
BT_DRIVE_AUTO = 4
BT_STEER_DRIVE_MANUAL = 5
BT_STEER_MANUAL = 6
BT_DRIVE_MANUAL = 7
BT_DRIVE_TORQUE = 8
BT_DRIVE_VELOCITY = 9

TX_DMERGENCY = 100

class Microbus_Can_Sender:
	def click_emegency_reset(self, event):
		msg = Empty()
		self.pub_emergency_reset.publish(msg)

	def click_steer_drive_auto(self, event):
		msg1 = Bool(True);
		msg2 = Bool(True);
		self.pub_steer_mode.publish(msg1)
		self.pub_drive_mode.publish(msg2)

	def click_steer_auto(self, event):
		msg1 = Bool(True);
		self.pub_steer_mode.publish(msg1)

	def click_drive_auto(self, event):
		msg2 = Bool(True);
		self.pub_drive_mode.publish(msg2)

	def click_steer_drive_manual(self, event):
		msg1 = Bool(False);
		msg2 = Bool(False);
		self.pub_steer_mode.publish(msg1)
		self.pub_drive_mode.publish(msg2)

	def click_steer_manual(self, event):
		msg1 = Bool(False);
		self.pub_steer_mode.publish(msg1)

	def click_drive_manual(self, event):
		msg2 = Bool(False);
		self.pub_drive_mode.publish(msg2)

	def click_drive_torque(self, event):
		msg = Empty();
		self.pub_drive_torque.publish(msg)

	def click_drive_velocity(self, event):
		msg = Empty();
		self.pub_drive_velocity.publish(msg)

	def __init__(self):
		rospy.init_node('microbus_can_topic_sender', anonymous=True)
		app = wx.App()
		frame = wx.Frame(None, -1, 'Hello,World!',size=(1000,500))

		panel = wx.Panel(frame, wx.ID_ANY)
		panel.SetBackgroundColour('#AFAFAF')
		font = wx.Font(20, wx.FONTFAMILY_DEFAULT,wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

		bt_emergency_reset = wx.Button(panel, BT_EMERGENCY_RESET, label="安全機能解除", pos=(10,10), size=(180,50))
		bt_emergency_reset.SetFont(font)
		bt_steer_drive_auto = wx.Button(panel, BT_STEER_DRIVE_AUTO, label="ステア、ドライブ\nAUTO", pos=(10,100), size=(230,80))
		bt_steer_drive_auto.SetFont(font)	
		bt_steer_auto = wx.Button(panel, BT_STEER_AUTO, label="ステア\nAUTO", pos=(250,100), size=(180,80))
		bt_steer_auto.SetFont(font)
		bt_drive_auto = wx.Button(panel, BT_DRIVE_AUTO, label="ドライブ\nAUTO", pos=(440,100), size=(180,80))
		bt_drive_auto.SetFont(font)
		bt_steer_drive_manual = wx.Button(panel, BT_STEER_DRIVE_MANUAL, label="ステア、ドライブ\nMANUAL", pos=(10,200), size=(230,80))
		bt_steer_drive_manual.SetFont(font)
		bt_steer_manual = wx.Button(panel, BT_STEER_MANUAL, label="ステア\nMANUAL", pos=(250,200), size=(180,80))
		bt_steer_manual.SetFont(font)
		bt_drive_manual = wx.Button(panel, BT_DRIVE_MANUAL, label="ドライブ\nMANUAL", pos=(440,200), size=(180,80))
		bt_drive_manual.SetFont(font)
		bt_drive_torque = wx.Button(panel, BT_DRIVE_TORQUE, label="TORQUE\nモード", pos=(10,300), size=(180,80))
		bt_drive_torque.SetFont(font)
		bt_drive_velocity = wx.Button(panel, BT_DRIVE_VELOCITY, label="VELOCITY\nモード", pos=(200,300), size=(180,80))
		bt_drive_velocity.SetFont(font)

		lb_auto_mode = wx.StaticText(panel, -1, '安全機能', pos=(670, 10))
		lb_auto_mode.SetFont(font)
		self.tx_auto_mode = wx.TextCtrl(panel, TX_DMERGENCY, pos=(790,14), style=wx.TE_PROCESS_ENTER)
		self.tx_auto_mode.SetFont(font)

		bt_emergency_reset.Bind(wx.EVT_BUTTON, self.click_emegency_reset)
		bt_steer_drive_auto.Bind(wx.EVT_BUTTON, self.click_steer_drive_auto)
		bt_steer_auto.Bind(wx.EVT_BUTTON, self.click_steer_auto)
		bt_drive_auto.Bind(wx.EVT_BUTTON, self.click_drive_auto)
		bt_steer_drive_manual.Bind(wx.EVT_BUTTON, self.click_steer_drive_manual)
		bt_steer_manual.Bind(wx.EVT_BUTTON, self.click_steer_manual)
		bt_drive_manual.Bind(wx.EVT_BUTTON, self.click_drive_manual)
		bt_drive_torque.Bind(wx.EVT_BUTTON, self.click_drive_torque)
		bt_drive_velocity.Bind(wx.EVT_BUTTON, self.click_drive_velocity)

		self.pub_emergency_reset = rospy.Publisher('/microbus/emergency_reset', Empty, queue_size=1)
		self.pub_steer_mode = rospy.Publisher('/microbus/steer_mode_send', Bool, queue_size=1)
		self.pub_drive_mode = rospy.Publisher('/microbus/drive_mode_send', Bool, queue_size=1)
		self.pub_drive_torque = rospy.Publisher('/microbus/set_torque_mode', Empty, queue_size=1)
		self.pub_drive_velocity = rospy.Publisher('/microbus/set_velocity_mode', Empty, queue_size=1)
		frame.Show()
		app.MainLoop()

if __name__ == '__main__':
	microbus_can_sender = Microbus_Can_Sender()
