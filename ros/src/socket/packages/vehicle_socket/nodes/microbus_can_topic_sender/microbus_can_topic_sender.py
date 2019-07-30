#!/usr/bin/python
# -*- coding: utf-8 -*-

import wx
import rospy
import threading
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import UInt8
from autoware_can_msgs.msg import MicroBusCan501
from autoware_can_msgs.msg import MicroBusCan502
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
BT_SHIFT_AUTO = 10
BT_SHIFT_MANUAL = 11
BT_SHIFT_P = 12
BT_SHIFT_R = 13
BT_SHIFT_N = 14
BT_SHIFT_D = 15
BT_SHIFT_2 = 16
BT_SHIFT_L = 17
BT_ENGINE_START = 18
BT_IGNITION = 19
BT_SIDE_BRAKE_ON = 20
BT_SIDE_BRAKE_OFF = 21
BT_AUTOMATIC_DOOR_OPEN = 22
BT_AUTOMATIC_DOOR_CLOSE = 23

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
TX_VELOCITY_ACTUAL = 207
TX_PEDAL_ACTUAL = 208
TX_ANGLE_ACTUAL = 209
TX_SHIFT = 210
TX_EMERGENCY_STOP = 211
TX_ENGINE_START = 212
TX_IGNITION = 213
TX_WIPER = 214
TX_LIGHT_HIGH = 215
TX_LIGHT_LOW = 216
TX_LIGHT_SMALL = 217
TX_HORN = 218
TX_HAZARD = 219
TX_BLINKER_RIGHT = 220
TX_BLINKER_LEFT = 221

BT_WIPER = 300
BT_LIGHT_HIGH = 301
BT_LIGHT_LOW = 302
BT_LIGHT_SMALL = 303
BT_HORN = 304
BT_HAZARD = 305
BT_BLINKER_RIGHT = 306
BT_BLINKER_LEFT = 307
BT_EMERGENCY_STOP = 308
BT_BLINKER_STOP = 309

class ReceiveValue:
	def __init__(self):
		self.emergency = True
		self.steer_on = False
		self.drive_on = False
		self.Dmode = MicroBusCan501.DRIVE_MODE_STROKE
		self.velocity = 0
		self.v_actual = 0
		self.pedal = 0
		self.angle = 0
		self.a_actual = 0
		self.input_drive = False
		self.input_steer = False
		self.shift = MicroBusCan501.SHIFT_MANUAL
		self.emergency_stop = 0
		self.engine_start = False
		self.ignition = False
		self.wiper = False
		self.light_high = False
		self.light_low = False
		self.light_small = False
		self.horn = False
		self.hazard = False
		self.blinker_right = False
		self.blinker_left = False
		self.blinker_stop = False

class Microbus_Can_Sender_GUI:
	def click_emergency_stop(self, event):
		msg = UInt8(2)
		self.pub_emergency_stop.publish(msg)

	def click_emegency_reset(self, event):
		msg = Empty()
		self.pub_emergency_reset.publish(msg)

	def click_engine_start(self, event):
		if self.receive.engine_start == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_engine_start.publish(msg)

	def click_ignition(self, event):
		if self.receive.ignition == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_ignition.publish(msg)

	def click_side_brake_on(self, event):
		msg = UInt8(2)
		self.pub_side_brake.publish(msg)

	def click_side_brake_off(self, event):
		msg = UInt8(1)
		self.pub_side_brake.publish(msg)

	def click_automatic_door_open(self, event):
		msg = UInt8(2)
		self.pub_automatic_door.publish(msg)

	def click_automatic_door_close(self, event):
		msg = UInt8(1)
		self.pub_automatic_door.publish(msg)

	def click_wiper(self, event):
		if self.receive.wiper == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_wiper.publish(msg)

	def click_light_high(self, event):
		if self.receive.light_high == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_light_high.publish(msg)

	def click_light_low(self, event):
		if self.receive.light_low == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_light_low.publish(msg)

	def click_light_small(self, event):
		if self.receive.light_small == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_light_small.publish(msg)

	def click_horn(self, event):
		if self.receive.horn == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_horn.publish(msg)

	def click_hazard(self, event):
		if self.receive.hazard == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_hazard.publish(msg)

	def click_blinker_right(self, event):
		if self.receive.blinker_right == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_blinker_right.publish(msg)

	def click_blinker_left(self, event):
		if self.receive.blinker_left == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_blinker_left.publish(msg)

	def click_blinker_stop(self, event):
		if self.receive.blinker_stop == False:
			msg = Bool(True)
		else:
			msg = Bool(False)
		self.pub_blinker_stop.publish(msg)

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

	def click_shift_auto(self, event):
		msg = Bool(True)
		self.pub_shift_auto.publish(msg)

	def click_shift_manual(self, event):
		msg = Bool(False)
		self.pub_shift_auto.publish(msg)

	def click_shift_P(self, event):
		msg = UInt8(0)
		self.pub_shift_position.publish(msg)

	def click_shift_R(self, event):
		msg = UInt8(1)
		self.pub_shift_position.publish(msg)

	def click_shift_N(self, event):
		msg = UInt8(2)
		self.pub_shift_position.publish(msg)

	def click_shift_D(self, event):
		msg = UInt8(3)
		self.pub_shift_position.publish(msg)

	def click_shift_2(self, event):
		msg = UInt8(4)
		self.pub_shift_position.publish(msg)

	def click_shift_L(self, event):
		msg = UInt8(5)
		self.pub_shift_position.publish(msg)

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

	def callback_can_receive501(self, msg):
		if msg.emergency == True:
			#self.tx_emergency.SetValue('LOCK')
			#self.tx_steer_auto.SetValue('NONE')
			#self.tx_drive_auto.SetValue('NONE')
			#self.tx_drive_mode.SetValue('NONE')
			self.receive.emergency = True
			self.receive.steer_on = False
			self.receive.drive_on = False
			self.receive.Dmode = MicroBusCan501.DRIVE_MODE_NONE
		else:
			#self.tx_emergency.SetValue('UNLOCK')
			self.receive.emergency = False
			if msg.steer_auto == True:
				#self.tx_steer_auto.SetValue('AUTO')
				self.receive.steer_on = True
			else:
				#self.tx_steer_auto.SetValue('MANUAL')
				self.receive.steer_on = False

			if msg.drive_auto == True:
				#self.tx_drive_auto.SetValue('AUTO')
				self.receive.drive_on = True

				if msg.drive_mode == MicroBusCan501.DRIVE_MODE_STROKE:
					#str_dmode = 'STROKE'
					self.receive.Dmode = MicroBusCan501.DRIVE_MODE_STROKE
				elif msg.drive_mode == MicroBusCan501.DRIVE_MODE_VELOCITY:
					#str_dmode = 'VELOCITY'
					self.receive.Dmode = MicroBusCan501.DRIVE_MODE_VELOCITY
				else:
					#str_dmode = 'UNKNOWN'
					self.receive.Dmode = MicroBusCan501.DRIVE_MODE_NONE
				#self.tx_drive_mode.SetValue(str_dmode)
			else:
				#self.tx_drive_auto.SetValue('MANUAL')
				self.receive.drive_on = False
				#self.tx_drive_mode.SetValue('UNKNOWN')
				self.receive.Dmode = MicroBusCan501.DRIVE_MODE_NONE

		#self.tx_velocity.SetValue(str(msg.velocity))
		self.receive.velocity = msg.velocity
		#self.tx_pedal.SetValue(str(msg.pedal))
		self.receive.pedal = msg.pedal
		#self.tx_angle.SetValue(str(msg.steering_angle))
		self.receive.angle = msg.steering_angle

		self.receive.emergency_stop = msg.emergency_stop
		self.receive.engine_start = msg.engine_start
		self.receive.ignition = msg.ignition
		self.receive.wiper = msg.wiper
		self.receive.light_high = msg.light_high
		self.receive.light_low = msg.light_low
		self.receive.light_small = msg.light_small
		self.receive.horn = msg.horn
		self.receive.hazard = msg.hazard
		self.receive.blinker_right = msg.blinker_right
		self.receive.blinker_left = msg.blinker_left

	def callback_can_receive502(self, msg): 
		#print("aaa")
		#self.tx_velocity_actual.SetValue(str(msg.velocity_actual))
		self.receive.v_actual = msg.velocity_actual
		#self.tx_angle_actual.SetValue(str(msg.angle_actual))
		self.receive.a_actual = msg.angle_actual

	def callback_micro_sub_can_sender_status(self, msg):
		self.receive.input_steer = msg.use_input_steer
		self.receive.input_drive = msg.use_input_drive
		#if msg.use_input_steer == True:
			#self.bt_input_steer_send.Enable()
			#self.tx_input_steer_label.SetValue("ON")
		#else:
			#self.bt_input_steer_send.Disable()
			#self.tx_input_steer_label.SetValue("OFF")

		#if msg.use_input_drive == True:
			#self.bt_input_drive_send.Enable()
			#self.tx_input_drive_label.SetValue("ON")
		#else:
			#self.tx_input_drive_label.SetValue("OFF")
			#self.bt_input_drive_send.Disable()

	#def worker(self):
	#	self.app.MainLoop()

	def original_MainLoop(self):
		evtloop = wx.GUIEventLoop()
		evtloop = wx.EventLoop()
		wx.EventLoop.SetActive( evtloop)

		#"""Drives the main wx event loop."""

		while 1:
		    while evtloop.Pending(): # if there is at least one event to be processed
		        evtloop.Dispatch() # process one event
		    #~ time.sleep(0.10)  
		    
			evtloop.ProcessIdle()

			if self.receive.emergency == True:
				self.tx_emergency.SetValue('LOCK')
			else:
				self.tx_emergency.SetValue('UNLOCK')
			if self.receive.steer_on == True:
				self.tx_steer_auto.SetValue('AUTO')
			else:
				self.tx_steer_auto.SetValue('MANUAL')
			if self.receive.drive_on == True:
				self.tx_drive_auto.SetValue('AUTO')
			else:
				self.tx_drive_auto.SetValue('MANUAL')
 			if self.receive.Dmode == MicroBusCan501.DRIVE_MODE_STROKE:
				self.tx_drive_mode.SetValue('STROKE')
			elif self.receive.Dmode == MicroBusCan501.DRIVE_MODE_VELOCITY:
				self.tx_drive_mode.SetValue('VELOCITY')
			else:
				self.tx_drive_mode.SetValue('NONE')

			self.tx_velocity.SetValue(str(self.receive.velocity))
			self.tx_velocity_actual.SetValue(str(self.receive.v_actual))
			self.tx_pedal.SetValue(str(self.receive.pedal))
			self.tx_angle.SetValue(str(self.receive.angle))
			self.tx_angle_actual.SetValue(str(self.receive.a_actual))
			self.frame.Refresh()

			if self.receive.input_steer == True:
				self.bt_input_steer_send.Enable()
				self.tx_input_steer_label.SetValue("ON")
			else:
				self.bt_input_steer_send.Disable()
				self.tx_input_steer_label.SetValue("OFF")

			if self.receive.input_drive == True:
				self.bt_input_drive_send.Enable()
				self.tx_input_drive_label.SetValue("ON")
			else:
				self.tx_input_drive_label.SetValue("OFF")
				self.bt_input_drive_send.Disable()

			if self.receive.shift == MicroBusCan501.SHIFT_MANUAL:
				self.tx_shift.SetValue('MANUAL')
			elif self.receive.shift == MicroBusCan501.SHIFT_NOW_CHANGE:
				self.tx_shift.SetValue('NOW_CHANGE')
			elif self.receive.shift == MicroBusCan501.SHIFT_P:
				self.tx_shift.SetValue('P')
			elif self.receive.shift == MicroBusCan501.SHIFT_R:
				self.tx_shift.SetValue('R')
			elif self.receive.shift == MicroBusCan501.SHIFT_N:
				self.tx_shift.SetValue('N')
			elif self.receive.shift == MicroBusCan501.SHIFT_D:
				self.tx_shift.SetValue('D')
			elif self.receive.shift == MicroBusCan501.SHIFT_2:
				self.tx_shift.SetValue('2')
			elif self.receive.shift == MicroBusCan501.SHIFT_L:
				self.tx_shift.SetValue('L')

			if self.receive.emergency_stop == 2:
				self.tx_emergency_stop.SetValue('停止')
			elif self.receive.emergency_stop == 1:
				self.tx_emergency_stop.SetValue('動作不良')
			else:
				self.tx_emergency_stop.SetValue('OFF')

			if self.receive.engine_start == True:
				self.tx_engine_start.SetValue('ON')
				self.bt_engine_start.SetLabel('エンジンOFF')
			else:
				self.tx_engine_start.SetValue('OFF')
				self.bt_engine_start.SetLabel('エンジンON')

			if self.receive.ignition == True:
				self.tx_ignition.SetValue('ON')
				self.bt_ignition.SetLabel('イグニションOFF')
			else:
				self.tx_ignition.SetValue('OFF')
				self.bt_ignition.SetLabel('イグニションON')

			if self.receive.wiper == True:
				self.tx_wiper.SetValue('ON')
			else:
				self.tx_wiper.SetValue('OFF')

			if self.receive.light_high == True:
				self.tx_light_high.SetValue('ON')
			else:
				self.tx_light_high.SetValue('OFF')

			if self.receive.light_low == True:
				self.tx_light_low.SetValue('ON')
			else:
				self.tx_light_low.SetValue('OFF')

			if self.receive.light_small == True:
				self.tx_light_small.SetValue('ON')
			else:
				self.tx_light_small.SetValue('OFF')

			if self.receive.horn == True:
				self.tx_horn.SetValue('ON')
			else:
				self.tx_horn.SetValue('OFF')

			if self.receive.hazard == True:
				self.tx_hazard.SetValue('ON')
			else:
				self.tx_hazard.SetValue('OFF')

			if self.receive.blinker_right == True:
				self.tx_blinker_right.SetValue('ON')
			else:
				self.tx_blinker_right.SetValue('OFF')

			if self.receive.blinker_left == True:
				self.tx_blinker_left.SetValue('ON')
			else:
				self.tx_blinker_left.SetValue('OFF')

	def __init__(self):
		rospy.init_node('microbus_can_topic_sender', anonymous=True)
		self.app = wx.App()
		self.frame = wx.Frame(None, -1, 'Hello,World!',size=(1050,1000))

		self.panel = wx.Panel(self.frame, wx.ID_ANY)
		self.panel.SetBackgroundColour('#AFAFAF')
		font = wx.Font(20, wx.FONTFAMILY_DEFAULT,wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
		font_min = wx.Font(15, wx.FONTFAMILY_DEFAULT,wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

		self.receive = ReceiveValue()

		self.bt_emergency_reset = wx.Button(self.panel, BT_EMERGENCY_RESET, label="安全機能解除", pos=(10,10), size=(180,50))
		self.bt_emergency_reset.SetFont(font)
		self.bt_engine_start = wx.Button(self.panel, BT_ENGINE_START, label="エンジンON", pos=(200,10), size=(180,50))
		self.bt_engine_start.SetFont(font)
		self.bt_ignition = wx.Button(self.panel, BT_IGNITION, label="イグニションON", pos=(390,10), size=(230,50))
		self.bt_ignition.SetFont(font)
		self.bt_side_brake_on = wx.Button(self.panel, BT_SIDE_BRAKE_ON, label="サイドブレーキON", pos=(10,70), size=(190,50))
		self.bt_side_brake_on.SetFont(font_min)	
		self.bt_side_brake_off = wx.Button(self.panel, BT_SIDE_BRAKE_OFF, label="サイドブレーキOFF", pos=(210,70), size=(190,50))
		self.bt_side_brake_off.SetFont(font_min)
		self.bt_automatic_door_open = wx.Button(self.panel, BT_AUTOMATIC_DOOR_OPEN, label="ドアopen", pos=(410,70), size=(100,50))
		self.bt_automatic_door_open.SetFont(font_min)
		self.bt_automatic_door_close = wx.Button(self.panel, BT_AUTOMATIC_DOOR_CLOSE, label="ドアclose", pos=(520,70), size=(100,50))
		self.bt_automatic_door_close.SetFont(font_min)
		self.bt_steer_drive_auto = wx.Button(self.panel, BT_STEER_DRIVE_AUTO, label="ステア、ドライブ\nAUTO", pos=(10,130), size=(230,80))
		self.bt_steer_drive_auto.SetFont(font)	
		self.bt_steer_auto = wx.Button(self.panel, BT_STEER_AUTO, label="ステア\nAUTO", pos=(250,130), size=(180,80))
		self.bt_steer_auto.SetFont(font)
		self.bt_drive_auto = wx.Button(self.panel, BT_DRIVE_AUTO, label="ドライブ\nAUTO", pos=(440,130), size=(180,80))
		self.bt_drive_auto.SetFont(font)
		self.bt_steer_drive_manual = wx.Button(self.panel, BT_STEER_DRIVE_MANUAL, label="ステア、ドライブ\nMANUAL", pos=(10,210), size=(230,80))
		self.bt_steer_drive_manual.SetFont(font)
		self.bt_steer_manual = wx.Button(self.panel, BT_STEER_MANUAL, label="ステア\nMANUAL", pos=(250,210), size=(180,80))
		self.bt_steer_manual.SetFont(font)
		self.bt_drive_manual = wx.Button(self.panel, BT_DRIVE_MANUAL, label="ドライブ\nMANUAL", pos=(440,210), size=(180,80))
		self.bt_drive_manual.SetFont(font)
		self.bt_drive_stroke = wx.Button(self.panel, BT_DRIVE_STROKE, label="STROKE\nモード", pos=(10,300), size=(180,80))
		self.bt_drive_stroke.SetFont(font)
		self.bt_drive_velocity = wx.Button(self.panel, BT_DRIVE_VELOCITY, label="VELOCITY\nモード", pos=(200,300), size=(180,80))
		self.bt_drive_velocity.SetFont(font)
		self.bt_shift_auto = wx.Button(self.panel, BT_SHIFT_AUTO, label="shift\nauto", pos=(390,390), size=(100,100))
		self.bt_shift_auto.SetFont(font)
		self.bt_shift_manual = wx.Button(self.panel, BT_SHIFT_MANUAL, label="shift\nmanual", pos=(500,390), size=(100,100))
		self.bt_shift_manual.SetFont(font)
		self.bt_shift_P = wx.Button(self.panel, BT_SHIFT_P, label="P", pos=(10,390), size=(50,50))
		self.bt_shift_P.SetFont(font)
		self.bt_shift_R = wx.Button(self.panel, BT_SHIFT_R, label="R", pos=(70,390), size=(50,50))
		self.bt_shift_R.SetFont(font)
		self.bt_shift_N = wx.Button(self.panel, BT_SHIFT_N, label="N", pos=(130,390), size=(50,50))
		self.bt_shift_N.SetFont(font)
		self.bt_shift_D = wx.Button(self.panel, BT_SHIFT_D, label="D", pos=(190,390), size=(50,50))
		self.bt_shift_D.SetFont(font)
		self.bt_shift_2 = wx.Button(self.panel, BT_SHIFT_2, label="2", pos=(250,390), size=(50,50))
		self.bt_shift_2.SetFont(font)
		self.bt_shift_L = wx.Button(self.panel, BT_SHIFT_L, label="L", pos=(310,390), size=(50,50))
		self.bt_shift_L.SetFont(font)

		self.bt_emergency_reset.Bind(wx.EVT_BUTTON, self.click_emegency_reset)
		self.bt_engine_start.Bind(wx.EVT_BUTTON, self.click_engine_start)
		self.bt_ignition.Bind(wx.EVT_BUTTON, self.click_ignition)
		self.bt_side_brake_on.Bind(wx.EVT_BUTTON, self.click_side_brake_on)
		self.bt_side_brake_off.Bind(wx.EVT_BUTTON, self.click_side_brake_off)
		self.bt_automatic_door_open.Bind(wx.EVT_BUTTON, self.click_automatic_door_open)
		self.bt_automatic_door_close.Bind(wx.EVT_BUTTON, self.click_automatic_door_close)
		self.bt_steer_drive_auto.Bind(wx.EVT_BUTTON, self.click_steer_drive_auto)
		self.bt_steer_auto.Bind(wx.EVT_BUTTON, self.click_steer_auto)
		self.bt_drive_auto.Bind(wx.EVT_BUTTON, self.click_drive_auto)
		self.bt_steer_drive_manual.Bind(wx.EVT_BUTTON, self.click_steer_drive_manual)
		self.bt_steer_manual.Bind(wx.EVT_BUTTON, self.click_steer_manual)
		self.bt_drive_manual.Bind(wx.EVT_BUTTON, self.click_drive_manual)
		self.bt_drive_stroke.Bind(wx.EVT_BUTTON, self.click_drive_stroke)
		self.bt_drive_velocity.Bind(wx.EVT_BUTTON, self.click_drive_velocity)
		self.bt_shift_auto.Bind(wx.EVT_BUTTON, self.click_shift_auto)
		self.bt_shift_manual.Bind(wx.EVT_BUTTON, self.click_shift_manual)
		self.bt_shift_P.Bind(wx.EVT_BUTTON, self.click_shift_P)
		self.bt_shift_R.Bind(wx.EVT_BUTTON, self.click_shift_R)
		self.bt_shift_N.Bind(wx.EVT_BUTTON, self.click_shift_N)
		self.bt_shift_D.Bind(wx.EVT_BUTTON, self.click_shift_D)
		self.bt_shift_2.Bind(wx.EVT_BUTTON, self.click_shift_2)
		self.bt_shift_L.Bind(wx.EVT_BUTTON, self.click_shift_L)

		self.bt_input_steer_on = wx.Button(self.panel, BT_INPUT_STEER_ON, label="ステア入力\nON", pos=(10,450), size=(180,80))
		self.bt_input_steer_on.SetFont(font)
		self.bt_input_drive_on = wx.Button(self.panel, BT_INPUT_DRIVE_ON, label="ドライブ入力\nON", pos=(200,450), size=(180,80))
		self.bt_input_drive_on.SetFont(font)
		self.bt_input_steer_off = wx.Button(self.panel, BT_INPUT_STEER_OFF, label="ステア入力\nOFF", pos=(10, 540), size=(180,80))
		self.bt_input_steer_off.SetFont(font)
		self.bt_input_drive_off = wx.Button(self.panel, BT_INPUT_DRIVE_OFF, label="ドライブ入力\nOFF", pos=(200, 540), size=(180,80))
		self.bt_input_drive_off.SetFont(font)
		self.tx_input_steer_label = wx.TextCtrl(self.panel, TX_INPUT_STEER_LABEL, pos=(10, 625), size=(180,45), style=wx.TE_READONLY)
		#self.tx_input_steer_flag = wx.StaticText(self.panel, LB_INPUT_STEER, 'OFF', pos=(10, 530))
		self.tx_input_steer_label.SetFont(font)
		self.tx_input_drive_label = wx.TextCtrl(self.panel, TX_INPUT_DRIVE_LABEL, pos=(200, 625), size=(180,45), style=wx.TE_READONLY)
		#self.tx_input_drive_flag = wx.StaticText(self.panel, LB_INPUT_DRIVE, 'OFF', pos=(200, 530))
		self.tx_input_drive_label.SetFont(font)
		self.tx_input_steer = wx.TextCtrl(self.panel, TX_INPUT_STEER, pos=(10,670), size=(180,45), style=wx.TE_PROCESS_ENTER)
		self.tx_input_steer.SetFont(font)
		self.tx_input_drive = wx.TextCtrl(self.panel, TX_INPUT_DRIVE, pos=(200,670), size=(180,45), style=wx.TE_PROCESS_ENTER)
		self.tx_input_drive.SetFont(font)
		self.bt_input_steer_send = wx.Button(self.panel, BT_INPUT_STEER_SEND, label="ステア送信", pos=(10,715), size=(180,80))
		self.bt_input_steer_send.SetFont(font)
		self.bt_input_steer_send.Disable()
		self.bt_input_drive_send = wx.Button(self.panel, BT_INPUT_DRIVE_SEND, label="ドライブ送信", pos=(200,715), size=(180,80))
		self.bt_input_drive_send.SetFont(font)
		self.bt_input_drive_send.Disable()

		self.bt_input_steer_on.Bind(wx.EVT_BUTTON, self.click_input_steer_on)
		self.bt_input_drive_on.Bind(wx.EVT_BUTTON, self.click_input_drive_on)
		self.bt_input_steer_off.Bind(wx.EVT_BUTTON, self.click_input_steer_off)
		self.bt_input_drive_off.Bind(wx.EVT_BUTTON, self.click_input_drive_off)
		self.bt_input_steer_send.Bind(wx.EVT_BUTTON, self.click_input_steer_send)
		self.bt_input_drive_send.Bind(wx.EVT_BUTTON, self.click_input_drive_send)

		self.bt_emergency_stop = wx.Button(self.panel, BT_WIPER, label="緊急停止", pos=(420,550), size=(130,200))
		self.bt_emergency_stop.SetFont(font)
		self.bt_wiper = wx.Button(self.panel, BT_WIPER, label="ワイパー", pos=(10,810), size=(130,50))
		self.bt_wiper.SetFont(font)
		self.bt_light_high = wx.Button(self.panel, BT_LIGHT_HIGH, label="ライトH", pos=(150,810), size=(130,50))
		self.bt_light_high.SetFont(font)
		self.bt_light_low = wx.Button(self.panel, BT_LIGHT_LOW, label="ライトL", pos=(290,810), size=(130,50))
		self.bt_light_low.SetFont(font)
		self.bt_light_small = wx.Button(self.panel, BT_LIGHT_SMALL, label="ライトS", pos=(430,810), size=(130,50))
		self.bt_light_small.SetFont(font)
		self.bt_horn = wx.Button(self.panel, BT_HORN, label="ホーン", pos=(10,870), size=(130,50))
		self.bt_horn.SetFont(font)
		self.bt_hazard = wx.Button(self.panel, BT_HAZARD, label="ハザード", pos=(150,870), size=(130,50))
		self.bt_hazard.SetFont(font)
		self.bt_blinker_right = wx.Button(self.panel, BT_BLINKER_RIGHT, label="→", pos=(430,870), size=(130,50))
		self.bt_blinker_right.SetFont(font)
		self.bt_blinker_left = wx.Button(self.panel, BT_BLINKER_LEFT, label="←", pos=(290,870), size=(130,50))
		self.bt_blinker_left.SetFont(font)
		self.bt_blinker_stop = wx.Button(self.panel, BT_BLINKER_STOP, label="blink_stop", pos=(430,930), size=(130,50))
		self.bt_blinker_stop.SetFont(font_min)

		self.bt_emergency_stop.Bind(wx.EVT_BUTTON, self.click_emergency_stop)
		self.bt_wiper.Bind(wx.EVT_BUTTON, self.click_wiper)
		self.bt_light_high.Bind(wx.EVT_BUTTON, self.click_light_high)
		self.bt_light_low.Bind(wx.EVT_BUTTON, self.click_light_low)
		self.bt_light_small.Bind(wx.EVT_BUTTON, self.click_light_small)
		self.bt_horn.Bind(wx.EVT_BUTTON, self.click_horn)
		self.bt_hazard.Bind(wx.EVT_BUTTON, self.click_hazard)
		self.bt_blinker_right.Bind(wx.EVT_BUTTON, self.click_blinker_right)
		self.bt_blinker_left.Bind(wx.EVT_BUTTON, self.click_blinker_left)
		self.bt_blinker_stop.Bind(wx.EVT_BUTTON, self.click_blinker_stop)

		self.lb_emergency = wx.StaticText(self.panel, -1, '安全機能', pos=(670, 10))
		self.lb_emergency.SetFont(font)
		self.tx_emergency = wx.TextCtrl(self.panel, TX_EMERGENCY, pos=(830,10), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_emergency.SetFont(font)
		self.lb_steer_auto = wx.StaticText(self.panel, -1, 'ステア', pos=(670, 55))
		self.lb_steer_auto.SetFont(font)
		self.tx_steer_auto = wx.TextCtrl(self.panel, TX_STEER_AUTO, pos=(830,55), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_steer_auto.SetFont(font)
		self.lb_drive_auto = wx.StaticText(self.panel, -1, 'ドライブ', pos=(670, 100))
		self.lb_drive_auto.SetFont(font)
		self.tx_drive_auto = wx.TextCtrl(self.panel, TX_DRIVE_AUTO, pos=(830,100), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_drive_auto.SetFont(font)
		self.lb_drive_mode = wx.StaticText(self.panel, -1, 'Dモード', pos=(670, 145))
		self.lb_drive_mode.SetFont(font)
		self.tx_drive_mode = wx.TextCtrl(self.panel, TX_DRIVE_MODE, pos=(830,145), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_drive_mode.SetFont(font)
		self.lb_velocity = wx.StaticText(self.panel, -1, 'VELOCITY', pos=(670, 195))
		self.lb_velocity.SetFont(font)
		self.tx_velocity = wx.TextCtrl(self.panel, TX_VELOCITY, pos=(830,190), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_velocity.SetFont(font)
		self.lb_velocity_actual = wx.StaticText(self.panel, -1, 'V_ACTUAL', pos=(670, 240))
		self.lb_velocity_actual.SetFont(font)
		self.tx_velocity_actual = wx.TextCtrl(self.panel, TX_VELOCITY_ACTUAL, pos=(830,235), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_velocity_actual.SetFont(font)
		self.lb_pedal = wx.StaticText(self.panel, -1, 'PEDAL', pos=(670, 285))
		self.lb_pedal.SetFont(font)
		self.tx_pedal = wx.TextCtrl(self.panel, TX_PEDAL, pos=(830,280), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_pedal.SetFont(font)
		self.lb_angle = wx.StaticText(self.panel, -1, 'ANGLE', pos=(670, 330))
		self.lb_angle.SetFont(font)
		self.tx_angle = wx.TextCtrl(self.panel, TX_ANGLE, pos=(830,325), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_angle.SetFont(font)
		self.lb_angle_actual = wx.StaticText(self.panel, -1, 'A_ACTUAL', pos=(670, 375))
		self.lb_angle_actual.SetFont(font)
		self.tx_angle_actual = wx.TextCtrl(self.panel, TX_ANGLE_ACTUAL, pos=(830,370), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_angle_actual.SetFont(font)
		self.lb_shift = wx.StaticText(self.panel, -1, 'シフト', pos=(670, 420))
		self.lb_shift.SetFont(font)
		self.tx_shift = wx.TextCtrl(self.panel, TX_SHIFT, pos=(830,415), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_shift.SetFont(font)
		self.lb_emergency_stop = wx.StaticText(self.panel, -1, '緊急停止', pos=(670, 465))
		self.lb_emergency_stop.SetFont(font)
		self.tx_emergency_stop = wx.TextCtrl(self.panel, TX_EMERGENCY_STOP, pos=(830,460), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_emergency_stop.SetFont(font)
		self.lb_engine_start = wx.StaticText(self.panel, -1, 'エンジン', pos=(670, 510))
		self.lb_engine_start.SetFont(font)
		self.tx_engine_start = wx.TextCtrl(self.panel, TX_ENGINE_START, pos=(830,505), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_engine_start.SetFont(font)
		self.lb_ignition = wx.StaticText(self.panel, -1, 'ignition', pos=(670, 555))
		self.lb_ignition.SetFont(font)
		self.tx_ignition = wx.TextCtrl(self.panel, TX_IGNITION, pos=(830,550), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_ignition.SetFont(font)
		self.lb_wiper = wx.StaticText(self.panel, -1, 'ワイパー', pos=(670, 600))
		self.lb_wiper.SetFont(font)
		self.tx_wiper = wx.TextCtrl(self.panel, TX_WIPER, pos=(830,595), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_wiper.SetFont(font)
		self.lb_light_high = wx.StaticText(self.panel, -1, 'ライトH', pos=(670, 645))
		self.lb_light_high.SetFont(font)
		self.tx_light_high = wx.TextCtrl(self.panel, TX_LIGHT_HIGH, pos=(830,640), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_light_high.SetFont(font)
		self.lb_light_low = wx.StaticText(self.panel, -1, 'ライトL', pos=(670, 690))
		self.lb_light_low.SetFont(font)
		self.tx_light_low = wx.TextCtrl(self.panel, TX_LIGHT_LOW, pos=(830,685), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_light_low.SetFont(font)
		self.lb_light_small = wx.StaticText(self.panel, -1, 'ライトS', pos=(670, 735))
		self.lb_light_small.SetFont(font)
		self.tx_light_small = wx.TextCtrl(self.panel, TX_LIGHT_SMALL, pos=(830,730), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_light_small.SetFont(font)
		self.lb_horn = wx.StaticText(self.panel, -1, 'ホーン', pos=(670, 780))
		self.lb_horn.SetFont(font)
		self.tx_horn = wx.TextCtrl(self.panel, TX_HORN, pos=(830,775), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_horn.SetFont(font)
		self.lb_hazard = wx.StaticText(self.panel, -1, 'ハザード', pos=(670, 825))
		self.lb_hazard.SetFont(font)
		self.tx_hazard = wx.TextCtrl(self.panel, TX_HAZARD, pos=(830,820), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_hazard.SetFont(font)
		self.lb_blinker_right = wx.StaticText(self.panel, -1, 'ウィンカー右', pos=(670, 870))
		self.lb_blinker_right.SetFont(font)
		self.tx_blinker_right = wx.TextCtrl(self.panel, TX_BLINKER_RIGHT, pos=(830,865), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_blinker_right.SetFont(font)
		self.lb_blinker_left = wx.StaticText(self.panel, -1, 'ウィンカー左', pos=(670, 915))
		self.lb_blinker_left.SetFont(font)
		self.tx_blinker_left = wx.TextCtrl(self.panel, TX_BLINKER_LEFT, pos=(830,910), size=(200,45), style=wx.TE_PROCESS_ENTER)
		self.tx_blinker_left.SetFont(font)

		self.pub_emergency_reset = rospy.Publisher('/microbus/emergency_reset', Empty, queue_size=1)
		self.pub_engine_start = rospy.Publisher('/microbus/engine_start', Bool, queue_size=1)
		self.pub_ignition = rospy.Publisher('/microbus/ignition', Bool, queue_size=1)
		self.pub_side_brake = rospy.Publisher('/microbus/side_brake', UInt8, queue_size=1)
		self.pub_automatic_door = rospy.Publisher('/microbus/automatic_door', UInt8, queue_size=1)
		self.pub_steer_auto = rospy.Publisher('/microbus/steer_mode_send', Bool, queue_size=1)
		self.pub_drive_auto = rospy.Publisher('/microbus/drive_mode_send', Bool, queue_size=1)
		self.pub_drive_stroke = rospy.Publisher('/microbus/set_stroke_mode', Empty, queue_size=1)
		self.pub_drive_velocity = rospy.Publisher('/microbus/set_velocity_mode', Empty, queue_size=1)
		self.pub_input_steer_flag = rospy.Publisher('/microbus/input_steer_flag', Bool, queue_size=1)
		self.pub_input_drive_flag = rospy.Publisher('/microbus/input_drive_flag', Bool, queue_size=1)
		self.pub_input_steer_value = rospy.Publisher('/microbus/input_steer_value', Int16, queue_size=1)
		self.pub_input_drive_value = rospy.Publisher('/microbus/input_drive_value', Int16, queue_size=1)
		self.pub_shift_auto = rospy.Publisher('/microbus/shift_auto', Bool, queue_size=1)
		self.pub_shift_position = rospy.Publisher('/microbus/shift_position', UInt8, queue_size=1)
		self.pub_emergency_stop = rospy.Publisher('/microbus/emergency_stop', UInt8, queue_size=1)
		self.pub_wiper = rospy.Publisher('/microbus/wiper', Bool, queue_size=1)
		self.pub_light_high = rospy.Publisher('/microbus/light_high', Bool, queue_size=1)
		self.pub_light_low = rospy.Publisher('/microbus/light_low', Bool, queue_size=1)
		self.pub_light_small = rospy.Publisher('/microbus/light_small', Bool, queue_size=1)
		self.pub_horn = rospy.Publisher('/microbus/horn', Bool, queue_size=1)
		self.pub_hazard = rospy.Publisher('/microbus/hazard', Bool, queue_size=1)
		self.pub_blinker_right = rospy.Publisher('/microbus/blinker_right', Bool, queue_size=1)
		self.pub_blinker_left = rospy.Publisher('/microbus/blinker_left', Bool, queue_size=1)
		self.pub_blinker_stop = rospy.Publisher('/microbus/blinker_stop', Bool, queue_size=1)

		self.sub_micro_bus_can501 = rospy.Subscriber('/microbus/can_receive501', MicroBusCan501, self.callback_can_receive501)
		self.sub_micro_bus_can502 = rospy.Subscriber('/microbus/can_receive502', MicroBusCan502, self.callback_can_receive502)
		self.sub_micro_bus_can_sender_status = rospy.Subscriber('/microbus/can_sender_status', MicroBusCanSenderStatus, self.callback_micro_sub_can_sender_status)

		self.frame.Show()
		#self.app.MainLoop()
		#self.threads = []
		#self.t = threading.Thread(target=self.worker)
		#self.threads.append(self.t)
		#self.t.start()
		self.original_MainLoop()
		#rospy.spin()

if __name__ == '__main__':
	microbus_can_sender = Microbus_Can_Sender_GUI()
