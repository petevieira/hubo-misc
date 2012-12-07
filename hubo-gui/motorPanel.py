#!/usr/bin/python
import wx
import util

# Set wx.TextCtrl widget (text box) dimensions
widgetHeight = 25
textBoxWidth = 50
comboBoxWidth = 120
spinCtrlWidth = 80
toggleButtonWidth = 70
radioBoxWidth = 100
	

class Motor_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create main BoxSizer and put all the panels into it
		mainPanel = wx.Panel(self)
		

		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		subSizer = wx.BoxSizer(wx.HORIZONTAL)
		# Create panels
		self.ctrlPanel   = Control_Panel(mainPanel)
		self.statusPanel = Status_Panel(mainPanel)


		subSizer.Add(self.ctrlPanel, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(self.statusPanel, 0, wx.ALL|wx.EXPAND, 3)

		self.mainSizer.Add(wx.StaticText(mainPanel, wx.ID_ANY, 'Motor Panel'), 0, wx.CENTER, border=3)
		self.mainSizer.Add(subSizer, 0, wx.CENTER, border=3)
		mainPanel.SetSizerAndFit(self.mainSizer)

class Control_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create widgets Control section
		#
		# control sizer label
		panelLabel = wx.StaticText(self, wx.ID_ANY, 'Control')
		util.makeBold(panelLabel)
		# control mode widgets
		ctrlModeLabel = wx.StaticText(self, wx.ID_ANY, '    Control Mode:')
		self.ctrlModeBox = wx.ComboBox(self, wx.ID_ANY, '', size=wx.Size(comboBoxWidth, widgetHeight), choices=['Open Loop','Position','Velocity','Current'])
		# reference input widgets
		refInputLabel = wx.StaticText(self, wx.ID_ANY, 'Reference Input:')
		self.refInputSpinCtrl = wx.SpinCtrl(self, wx.ID_ANY, '', size=wx.Size(spinCtrlWidth, widgetHeight))
		# h-brdige setting widgets
		hBridgeLabel = wx.StaticText(self, wx.ID_ANY, '                H-Bridge:')
		self.hBridgeButton = wx.ToggleButton(self, wx.ID_ANY, 'ON/OFF', size=wx.Size(toggleButtonWidth, widgetHeight))
		# servo feedback widgets
		servoFbkLabel = wx.StaticText(self, wx.ID_ANY, 'Servo Feedback:')
		self.servoFbkButton = wx.ToggleButton(self, wx.ID_ANY, 'ON/OFF', size=wx.Size(toggleButtonWidth, widgetHeight))
		# rotate motor widgets
		rotateLabel = wx.StaticText(self, wx.ID_ANY, '     Rotate Motor:')
		self.rotateLeftButton = wx.Button(self, wx.ID_ANY, '<', size=wx.Size(toggleButtonWidth/2, widgetHeight))
		self.rotateRightButton = wx.Button(self, wx.ID_ANY, '>', size=wx.Size(toggleButtonWidth/2, widgetHeight))

		# Create sizers
		#
		# Create main BoxSizer for panel
		self.mainSizer = wx.BoxSizer(wx.VERTICAL)

		# Create box sizers for Control section
		self.ctrlModeSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.refInputSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.hBridgeSizer  = wx.BoxSizer(wx.HORIZONTAL)
		self.servoFbkSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.rotateSizer   = wx.BoxSizer(wx.HORIZONTAL)

		# Create box sizers State section


		# Add widgets to respective sizers
		#
		# Add widgets to ctrlModeSizer
		self.ctrlModeSizer.Add(ctrlModeLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.ctrlModeSizer.Add(self.ctrlModeBox, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to refInputSizer
		self.refInputSizer.Add(refInputLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.refInputSizer.Add(self.refInputSpinCtrl, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to hBridgeSizer
		self.hBridgeSizer.Add(hBridgeLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.hBridgeSizer.Add(self.hBridgeButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to servoFbkSizer
		self.servoFbkSizer.Add(servoFbkLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.servoFbkSizer.Add(self.servoFbkButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to rotateSizer
		self.rotateSizer.Add(rotateLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.rotateSizer.Add(self.rotateLeftButton, 0, wx.ALIGN_CENTER)
		self.rotateSizer.Add(self.rotateRightButton, 0, wx.ALIGN_CENTER)

		# Add all sizers to main sizer
		self.mainSizer.Add(panelLabel, 0, wx.CENTER, 3)
		self.mainSizer.Add(self.ctrlModeSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.refInputSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.hBridgeSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.servoFbkSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.rotateSizer, 0, wx.ALL|wx.EXPAND, 3)
	
		# Set sizer for panel
		self.SetSizer(self.mainSizer)


class Status_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create widgets Control section
		#
		# control sizer label
		panelLabel = wx.StaticText(self, wx.ID_ANY, 'Status')
		util.makeBold(panelLabel)
		# control mode widgets
		encoderLabel = wx.StaticText(self, wx.ID_ANY, 'Encoder Position:')
		self.encoderValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# reference input widgets
		currentLabel = wx.StaticText(self, wx.ID_ANY, '              Current (A):')
		self.currentValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# h-brdige setting widgets
		thermalLabel = wx.StaticText(self, wx.ID_ANY, ' Thermal Load (J):')
		self.thermalValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# servo feedback widgets
		limitLabel = wx.StaticText(self, wx.ID_ANY, '           Limit Switch:')
		self.limitValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))


		# Create sizers
		#
		# Create main BoxSizer for panel
		mainSizer = wx.BoxSizer(wx.VERTICAL)

		# Create box sizers for section
		encoderSizer = wx.BoxSizer(wx.HORIZONTAL)
		currentSizer = wx.BoxSizer(wx.HORIZONTAL)
		thermalSizer = wx.BoxSizer(wx.HORIZONTAL)
		limitSizer   = wx.BoxSizer(wx.HORIZONTAL)


		# Add widgets to respective sizers
		#
		# Add widgets to encoderSizer
		encoderSizer.Add(encoderLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		encoderSizer.Add(self.encoderValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to currentSizer
		currentSizer.Add(currentLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		currentSizer.Add(self.currentValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to thermalSizer
		thermalSizer.Add(thermalLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		thermalSizer.Add(self.thermalValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to limitSizer
		limitSizer.Add(limitLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		limitSizer.Add(self.limitValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add all sizers to main sizer
		mainSizer.Add(panelLabel, 0, wx.CENTER, 3)
		mainSizer.Add(encoderSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(currentSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(thermalSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(limitSizer, 0, wx.ALL|wx.EXPAND, 3)
	
		# Set sizer for panel
		self.SetSizer(mainSizer)


