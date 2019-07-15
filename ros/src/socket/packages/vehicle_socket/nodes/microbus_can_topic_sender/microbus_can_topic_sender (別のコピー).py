#!/usr/bin/python
# -*- coding: utf-8 -*-

import wx
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from autoware_can_msgs.msg import MicroBusCan
from autoware_can_msgs.msg import MicroBusCanSenderStatus

BT_EMERGENCY_RESET = 1
BT_STEER_DRIVE_AUTO = 2
BT_STEER_AUTO = 3
BT_DRIVE_AUTO = 4
BT_STEER_DRIVE_MANUAL = 5
BT_STEER_MANUAL = 6
BT_DRIVE_MANUAL = 7
BT_DRIVE_STROKE = 8
BT_DRIVE_VELOCITY = 9

BT_INPUT_STEER_ON = 100
BT_INPUT_DRIVE_ON = 101
BT_INPUT_STEER_OFF = 102
BT_INPUT_DRIVE_OFF = 103
TX_INPUT_STEER_LABEL = 104
TX_INPUT_DRIVE_LABEL  = 105
TX_INPUT_STEER = 106
TX_INPUT_DRIVE = 107
BT_INPUT_STEER_SEND = 108
BT_INPUT_DRIVE_SEND = 109

TX_EMERGENCY = 200
TX_STEER_AUTO = 201
TX_DRIVE_AUTO = 202
TX_DRIVE_MODE = 203
TX_VELOCITY = 204
TX_PEDAL = 205
TX_ANGLE = 206

class Microbus_Can_Sender:
	def click_emegency_reset(self, event):
		msg = Empty()
		self.pub_emergency_reset.publish(msg)

	def click_steer_drive_auto(self, event):
		msg1 = Bool(True)
		msg2 = Bool(True)
		self.pub_steer_auto.publish(msg1)
		self.pub_drive_auto.publish(msg2)

	def click_steer_auto(self, event):
		msg1 = Bool(True)
		self.pub_steer_auto.publish(msg1)

	def click_drive_auto(self, event):
		msg2 = Bool(True)
		self.pub_drive_auto.publish(msg2)

	def click_steer_drive_manual(self, event):
		msg1 = Bool(False)
		msg2 = Bool(False)
		self.pub_steer_auto.publish(msg1)
		self.pub_drive_auto.publish(msg2)

	def click_steer_manual(self, event):
		msg1 = Bool(False);
		self.pub_steer_auto.publish(msg1)

	def click_drive_manual(self, event):
		msg2 = Bool(False)
		self.pub_drive_auto.publish(msg2)

	def click_drive_stroke(self, event):
		msg = Empty()
		self.pub_drive_stroke.publish(msg)

	def click_drive_velocity(self, event):
		msg = Empty()
		self.pub_drive_velocity.publish(msg)

	def click_input_steer_on(self, event):
		msg = Bool(True)
		self.pub_input_steer_flag.publish(msg)

	def click_input_steer_off(self, event):
		msg = Bool(False)
		self.pub_input_steer_flag.publish(msg)

	def click_input_drive_on(self, event):
		msg = Bool(True)
		self.pub_input_drive_flag.publish(msg)

	def click_input_drive_off(self, event):
		msg = Bool(False)
		self.pub_input_drive_flag.publish(msg)

	def click_input_steer_send(self, event):
		str = self.tx_input_steer.GetValue()
		val = int(str)
		msg = Int16(val)
		self.pub_input_steer_value.publish(msg)

	def click_input_drive_send(self, event):
		str = self.tx_input_drive.GetValue()
		val = int(str)
		msg = Int16(val)
		self.pub_input_drive_value.publish(msg)

	def callback_can_receive(self, msg):
		if msg.emergency == True:
			self.tx_emergency.SetValue('LOCK')
			self.tx_steer_auto.SetValue('NONE')
			self.tx_drive_auto.SetValue('NONE')
			self.tx_drive_mode.SetValue('NONE')
		else:
			self.tx_emergency.SetValue('UNLOCK')
			if msg.steer_auto == True:
				self.tx_steer_auto.SetValue('AUTO')
			else:
				self.tx_steer_auto.SetValue('MANUAL')

			if msg.drive_auto == True:
				self.tx_drive_auto.SetValue('AUTO')

				if msg.drive_mode == MicroBusCan.DRIVE_MODE_STROKE:
					str_dmode = 'STROKE'
				elif msg.drive_mode == MicroBusCan.DRIVE_MODE_VELOCITY:
					str_dmode = 'VELOCITY'
				else:
					str_dmode = 'UNKNOWN'
				self.tx_drive_mode.SetValue(str_dmode)

				#if str_dmode != 'UNKNOWN':
					#self.tx_velocity.SetValue(str(msg.velocity))
					#self.tx_pedal.SetValue(str(msg.pedal))
					#self.tx_angle.SetValue(str(msg.steering_angle))
				#else:
					#self.tx_velocity.SetValue('NONE')
					#self.tx_pedal.SetValue('NONE')
					#self.tx_angle.SetValue('NONE')
			else:
				self.tx_drive_auto.SetValue('MANUAL')
				self.tx_drive_mode.SetValue('UNKNOWN')
				#self.tx_velocity.SetValue('NONE')
				#self.tx_pedal.SetValue('NONE')
				#self.tx_angle.SetValue('NONE')
		self.tx_velocity.SetValue(str(msg.velocity))
		self.tx_pedal.SetValue(str(msg.pedal))
		self.tx_angle.SetValue(str(msg.steering_angle))

	def callback_micro_sub_can_sender_status(self, msg):
		if msg.use_input_steer == True:
			self.tx_input_steer_label.SetValue('ON')
			self.bt_input_steer_send.Enable()
		else:
			self.tx_input_steer_label.SetValue('OFF')
			self.bt_input_steer_send.Disable()

		if msg.use_input_drive == True:
			self.tx_input_drive_label.SetValue('ON')
			self.bt_input_drive_send.Enable()
		else:
			self.tx_input_drive_label.SetValue('OFF')
			self.bt_input_drive_send.Disable()

	def __init__(self):
		rospy.init_node('microbus_can_topic_sender', anonymous=True)
		self.app = wx.App()
		self.frame = wx.Frame(None, -1, 'Hello,World!',size=(1050,800))

		self.panel = wx.Panel(self.frame, wx.ID_ANY)
		self.panel.SetBackgroundColour('#AFAFAF')
		font = wx.Font(20, wx.FONTFAMILY_DEFAULT,wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

		self.bt_emergency_reset = wx.Button(self.panel, BT_EMERGENCY_RESET, label="安全機能解除", pos=(10,10), size=(180,50))
		self.bt_emergency_reset.SetFont(font)
		self.bt_steer_drive_auto = wx.Button(self.panel, BT_STEER_DRIVE_AUTO, label="ステア、ドライブ\nAUTO", pos=(10,100), size=(230,80))
		self.bt_steer_drive_auto.SetFont(font)	
		self.bt_steer_auto = wx.Button(self.panel, BT_STEER_AUTO, label="ステア\nAUTO", pos=(250,100), size=(180,80))
		self.bt_steer_auto.SetFont(font)
		self.bt_drive_auto = wx.Button(self.panel, BT_DRIVE_AUTO, label="ドライブ\nAUTO", pos=(440,100), size=(180,80))
		self.bt_drive_auto.SetFont(font)
		self.bt_steer_drive_manual = wx.Button(self.panel, BT_STEER_DRIVE_MANUAL, label="ステア、ドライブ\nMANUAL", pos=(10,200), size=(230,80))
		self.bt_steer_drive_manual.SetFont(font)
		self.bt_steer_manual = wx.Button(self.panel, BT_STEER_MANUAL, label="ステア\nMANUAL", pos=(250,200), size=(180,80))
		self.bt_steer_manual.SetFont(font)
		self.bt_drive_manual = wx.Button(self.panel, BT_DRIVE_MANUAL, label="ドライブ\nMANUAL", pos=(440,200), size=(180,80))
		self.bt_drive_manual.SetFont(font)
		self.bt_drive_stroke = wx.Button(self.panel, BT_DRIVE_STROKE, label="STROKE\nモード", pos=(10,300), size=(180,80))
		self.bt_drive_stroke.SetFont(font)
		self.bt_drive_velocity = wx.Button(self.panel, BT_DRIVE_VELOCITY, label="VELOCITY\nモード", pos=(200,300), size=(180,80))
		self.bt_drive_velocity.SetFont(font)

		self.bt_emergency_reset.Bind(wx.EVT_BUTTON, self.click_emegency_reset)
		self.bt_steer_drive_auto.Bind(wx.EVT_BUTTON, self.click_steer_drive_auto)
		self.bt_steer_auto.Bind(wx.EVT_BUTTON, self.click_steer_auto)
		self.bt_drive_auto.Bind(wx.EVT_BUTTON, self.click_drive_auto)
		self.bt_steer_drive_manual.Bind(wx.EVT_BUTTON, self.click_steer_drive_manual)
		self.bt_steer_manual.Bind(wx.EVT_BUTTON, self.click_steer_manual)
		self.bt_drive_manual.Bind(wx.EVT_BUTTON, self.click_drive_manual)
		self.bt_drive_stroke.Bind(wx.EVT_BUTTON, self.click_drive_stroke)
		self.bt_drive_velocity.Bind(wx.EVT_BUTTON, self.click_drive_velocity)

		self.bt_input_steer_on = wx.Button(self.panel, BT_INPUT_STEER_ON, label="ステア入力\nON", pos=(10,440), size=(180,80))
		self.bt_input_steer_on.SetFont(font)
		self.bt_input_drive_on = wx.Button(self.panel, BT_INPUT_DRIVE_ON, label="ドライブ入力\nON", pos=(200,440), size=(180,80))
		self.bt_input_drive_on.SetFont(font)
		self.bt_input_steer_off = wx.Button(self.panel, BT_INPUT_STEER_OFF, label="ステア入力\nOFF", pos=(10, 530), size=(180,80))
		self.bt_input_steer_off.SetFont(font)
		self.bt_input_drive_off = wx.Button(self.panel, BT_INPUT_DRIVE_OFF, label="ドライブ入力\nOFF", pos=(200, 530), size=(180,80))
		self.bt_input_drive_off.SetFont(font)
		self.tx_input_steer_label = wx.TextCtrl(self.panel, TX_INPUT_STEER_LABEL, pos=(10, 615), size=(180,45), style=wx.TE_PROCESS_ENTER)
		#self.tx_input_steer_flag = wx.StaticText(self.panel, LB_INPUT_STEER, 'OFF', pos=(10, 530))
		self.tx_input_steer_label.SetFont(font)
		self.tx_input_drive_label = wx.TextCtrl(self.panel, TX_INPUT_DRIVE_LABEL, pos=(200, 615), size=(180,45), style=wx.TE_PROCESS_ENTER)
		#self.tx_input_drive_flag = wx.StaticText(self.panel, LB_INPUT_DRIVE, 'OFF', pos=(200, 530))
		self.tx_input_drive_label.SetFont(font)
		self.tx_input_steer = wx.TextCtrl(self.panel, TX_INPUT_STEER, pos=(10,660), size=(180,45), style=wx.TE_PROCESS_ENTER)
		self.tx_input_steer.SetFont(font)
		self.tx_input_drive = wx.TextCtrl(self.panel, TX_INPUT_DRIVE, pos=(200,660), size=(180,45), style=wx.TE_PROCESS_ENTER)
		self.tx_input_drive.SetFont(font)
		self.bt_input_steer_send = wx.Button(self.panel, BT_INPUT_STEER_SEND, label="ステア送信", pos=(10,705), size=(180,80))
		self.bt_input_steer_send.SetFont(font)
		self.bt_input_steer_send.Disable()
		self.bt_input_drive_send = wx.Button(self.panel, BT_INPUT_DRIVE_SEND, label="ドライブ送信", pos=(200,705), size=(180,80))
		self.bt_input_drive_send.SetFont(font)
		self.bt_input_drive_send.Disable()

		self.bt_input_steer_on.Bind(wx.EVT_BUTTON, self.click_input_steer_on)
		self.bt_input_drive_on.Bind(wx.EVT_BUTTON, self.click_input_drive_on)
		self.bt_input_steer_off.Bind(wx.EVT_BUTTON, self.click_input_steer_off)
		self.bt_input_drive_off.Bind(wx.EVT_BUTTON, self.click_input_drive_off)
		self.bt_input_steer_send.Bind(wx.EVT_BUTTON, self.click_input_steer_send)
		self.bt_input_drive_send.Bind(wx.EVT_BUTTON, self.click_input_drive_send)

		self.lb_emergency = wx.StaticText(self.panel, -1, '安全機能', pos=(670, 10))
		self.lb_emergency.SetFont(font)
		self.tx_emergency = wx.TextCtrl(self.panel, TX_EMERGENCY, pos=(810,10), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_emergency.SetFont(font)
		self.lb_steer_auto = wx.StaticText(self.panel, -1, 'ステア', pos=(670, 55))
		self.lb_steer_auto.SetFont(font)
		self.tx_steer_auto = wx.TextCtrl(self.panel, TX_STEER_AUTO, pos=(810,55), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_steer_auto.SetFont(font)
		self.lb_drive_auto = wx.StaticText(self.panel, -1, 'ドライブ', pos=(670, 100))
		self.lb_drive_auto.SetFont(font)
		self.tx_drive_auto = wx.TextCtrl(self.panel, TX_DRIVE_AUTO, pos=(810,100), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_drive_auto.SetFont(font)
		self.lb_drive_mode = wx.StaticText(self.panel, -1, 'Dモード', pos=(670, 145))
		self.lb_drive_mode.SetFont(font)
		self.tx_drive_mode = wx.TextCtrl(self.panel, TX_DRIVE_MODE, pos=(810,145), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_drive_mode.SetFont(font)
		self.lb_velocity = wx.StaticText(self.panel, -1, 'VELOCITY', pos=(670, 195))
		self.lb_velocity.SetFont(font)
		self.tx_velocity = wx.TextCtrl(self.panel, TX_VELOCITY, pos=(810,190), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_velocity.SetFont(font)
		self.lb_pedal = wx.StaticText(self.panel, -1, 'PEDAL', pos=(670, 240))
		self.lb_pedal.SetFont(font)
		self.tx_pedal = wx.TextCtrl(self.panel, TX_PEDAL, pos=(810,235), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_pedal.SetFont(font)
		self.lb_angle = wx.StaticText(self.panel, -1, 'ANGLE', pos=(670, 285))
		self.lb_angle.SetFont(font)
		self.tx_angle = wx.TextCtrl(self.panel, TX_ANGLE, pos=(810,280), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_angle.SetFont(font)

		self.pub_emergency_reset = rospy.Publisher('/microbus/emergency_reset', Empty, queue_size=1)
		self.pub_steer_auto = rospy.Publisher('/microbus/steer_mode_send', Bool, queue_size=1)
		self.pub_drive_auto = rospy.Publisher('/microbus/drive_mode_send', Bool, queue_size=1)
		self.pub_drive_stroke = rospy.Publisher('/microbus/set_stroke_mode', Empty, queue_size=1)
		self.pub_drive_velocity = rospy.Publisher('/microbus/set_velocity_mode', Empty, queue_size=1)
		self.pub_input_steer_flag = rospy.Publisher('/microbus/input_steer_flag', Bool, queue_size=1)
		self.pub_input_drive_flag = rospy.Publisher('/microbus/input_drive_flag', Bool, queue_size=1)
		self.pub_input_steer_value = rospy.Publisher('/microbus/input_steer_value', Int16, queue_size=1)
		self.pub_input_drive_value = rospy.Publisher('/microbus/input_drive_value', Int16, queue_size=1)

		self.sub_micro_bus_can = rospy.Subscriber('/microbus/can_receive', MicroBusCan, self.callback_can_receive)
		self.sub_micro_bub_can_sender_status = rospy.Subscriber('/microbus/can_sender_status', MicroBusCanSenderStatus, self.callback_micro_sub_can_sender_status)
		
		self.frame.Show()
		self.app.MainLoop()

if __name__ == '__main__':
	microbus_can_sender = Microbus_Can_Sender()
	rospy.spin()
