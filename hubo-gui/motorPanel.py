#!/usr/bin/python
import wx
import util
import settings

# Set wx.TextCtrl widget (text box) dimensions
widgetHeight = 25
textBoxWidth = 50
comboBoxWidth = 110
spinCtrlWidth = 80
toggleButtonWidth = 70
radioBoxWidth = 100
	

class Motor_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create main BoxSizer and put all the panels into it
		mainPanel = wx.Panel(self)

		boardNumLabel = wx.StaticText(mainPanel, wx.ID_ANY, 'Board No:')
		self.boardNumValue = wx.TextCtrl(mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth/1.5, widgetHeight))
		self.boardChanValue = wx.TextCtrl(mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth/2, widgetHeight))
		self.jointValue = wx.TextCtrl(mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))

		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		boardSizer = wx.BoxSizer(wx.HORIZONTAL)
		subSizer = wx.BoxSizer(wx.HORIZONTAL)

		# Create panels
		self.commandsPanel = Commands_Panel(mainPanel)
		self.ctrlPanel   = Control_Panel(mainPanel)
		self.statusPanel = Status_Panel(mainPanel)
		self.errorsPanel = Errors_Panel(mainPanel)

		boardSizer.Add(boardNumLabel, 0, wx.ALL|wx.EXPAND, 3)
		boardSizer.Add(self.boardNumValue, 0, wx.ALL|wx.EXPAND, 3)
		boardSizer.Add(self.boardChanValue, 0, wx.ALL|wx.EXPAND, 3)
		boardSizer.Add(self.jointValue, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(self.commandsPanel, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(self.ctrlPanel, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(self.statusPanel, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(self.errorsPanel, 0, wx.ALL|wx.EXPAND, 3)

		self.mainSizer.Add(wx.StaticText(mainPanel, wx.ID_ANY, 'Motor Panel'), 0, wx.CENTER, border=3)
		self.mainSizer.Add(boardSizer, 0, wx.CENTER, border=3)
		self.mainSizer.Add(subSizer, 0, wx.CENTER, border=3)
		mainPanel.SetSizerAndFit(self.mainSizer)


class Commands_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create widgets Control section
		#
		# control sizer label
		panelLabel = wx.StaticText(self, wx.ID_ANY, 'Commands')
		util.makeBold(panelLabel)
		#
		# create column1Sizer widgets
		self.initButton = wx.Button(self, wx.ID_ANY, label='Initialize')
		self.initAllButton = wx.Button(self, wx.ID_ANY, label='Initialize All')
		self.homeButton = wx.Button(self, wx.ID_ANY, label='Home')
		self.homeAllButton = wx.Button(self, wx.ID_ANY, label='Home All')
		self.zeroEncButton = wx.Button(self, wx.ID_ANY, label='Zero Encoder')
		self.settingsButton = wx.Button(self, wx.ID_ANY, label='Settings')


		# Bind events to objects
		#
		self.Bind(wx.EVT_BUTTON, self.OnSettings, self.settingsButton)
		
 
		# Create sizers
		#
		# Create main BoxSizer for panel
		mainSizer = wx.BoxSizer(wx.VERTICAL)


		# Add widgets to respective sizers
		#
		mainSizer.Add(panelLabel, 0, wx.CENTER, 3)
		mainSizer.Add(self.initButton, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(self.initAllButton, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(self.homeButton, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(self.homeAllButton, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(self.zeroEncButton, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(self.settingsButton, 0, wx.ALL|wx.EXPAND, 3)

		# Set sizer for panel
		self.SetSizer(mainSizer)

	def OnSettings(self, event):
		self.settingsFrame = settings.Settings_Frame(None, wx.ID_ANY, 'Settings', size=wx.Size(600,600))
	#	self.settingsFrame.Fit()
		self.settingsFrame.Show()


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
		self.hBridgeButton = wx.ToggleButton(self, wx.ID_ANY, 'OFF', size=wx.Size(toggleButtonWidth, widgetHeight))
		# servo feedback widgets
		servoFbkLabel = wx.StaticText(self, wx.ID_ANY, 'Servo Feedback:')
		self.servoFbkButton = wx.ToggleButton(self, wx.ID_ANY, 'OFF', size=wx.Size(toggleButtonWidth, widgetHeight))
		# rotate motor widgets
		rotateLabel = wx.StaticText(self, wx.ID_ANY, '     Rotate Motor:')
		self.rotateLeftButton = wx.Button(self, wx.ID_ANY, '<', size=wx.Size(toggleButtonWidth/2, widgetHeight))
		self.rotateRightButton = wx.Button(self, wx.ID_ANY, '>', size=wx.Size(toggleButtonWidth/2, widgetHeight))


		self.Bind(wx.EVT_TOGGLEBUTTON, self.OnToggle, self.hBridgeButton)
		self.Bind(wx.EVT_TOGGLEBUTTON, self.OnToggle, self.servoFbkButton)


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

	def OnToggle(self, event):
		object =  event.GetEventObject()
	
		if object.GetValue() == False:
			object.SetLabel('OFF')

		elif object.GetValue() == True:
			object.SetLabel('ON')


class Status_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		degSymbol = u'\u00b0'

		# Create widgets Control section
		#
		# control sizer label
		panelLabel = wx.StaticText(self, wx.ID_ANY, 'Status')
		util.makeBold(panelLabel)
		# encoder position widgets
		encoderLabel = wx.StaticText(self, wx.ID_ANY, ' Encoder Position:')
		self.encoderValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# current value widgets
		currentLabel = wx.StaticText(self, wx.ID_ANY, '              Current (A):')
		self.currentValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# thermal load widgets
		thermalLabel = wx.StaticText(self, wx.ID_ANY, ' Thermal Load (J):')
		self.thermalValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# temperature widgets
		temperatureLabel = wx.StaticText(self, wx.ID_ANY, u' Temperature ({}C):'.format(degSymbol))
		self.temperatureValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		# limit switch widgets
		limitLabel = wx.StaticText(self, wx.ID_ANY, '            Limit Switch:')
		self.limitValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))


		# Create sizers
		#
		# Create main BoxSizer for panel
		mainSizer = wx.BoxSizer(wx.VERTICAL)

		# Create box sizers for section
		encoderSizer = wx.BoxSizer(wx.HORIZONTAL)
		currentSizer = wx.BoxSizer(wx.HORIZONTAL)
		thermalSizer = wx.BoxSizer(wx.HORIZONTAL)
		tempSizer = wx.BoxSizer(wx.HORIZONTAL)
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

		# Add widgets to tempSizer
		tempSizer.Add(temperatureLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		tempSizer.Add(self.temperatureValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to limitSizer
		limitSizer.Add(limitLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		limitSizer.Add(self.limitValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add all sizers to main sizer
		mainSizer.Add(panelLabel, 0, wx.CENTER, 3)
		mainSizer.Add(encoderSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(currentSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(thermalSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(tempSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(limitSizer, 0, wx.ALL|wx.EXPAND, 3)
	
		# Set sizer for panel
		self.SetSizer(mainSizer)


class Errors_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		emptyImage = wx.EmptyBitmap(0,0)
		# Create widgets Control section
		#
		# control sizer label
		panelLabel = wx.StaticText(self, wx.ID_ANY, 'Error Flags')
		util.makeBold(panelLabel)
		#
		# create column1Sizer widgets
		motor0FaultLabel = wx.StaticText(self, wx.ID_ANY, '           Motor 0 Fault:')
		self.motor0FaultFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		motor1FaultLabel = wx.StaticText(self, wx.ID_ANY, '           Motor 1 Fault:')
		self.motor1FaultFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		encoderLabel = wx.StaticText(self, wx.ID_ANY, '          Encoder Error:')
		self.encoderFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)
	
		bigInputLabel = wx.StaticText(self, wx.ID_ANY, '         Big Input Error:')
		self.bigInputFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		motorJamLabel = wx.StaticText(self, wx.ID_ANY, '    Motor Jam Error:')
		self.motorJamFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		pwmSatLabel = wx.StaticText(self, wx.ID_ANY, 'PWM Output Error:')
		self.pwmSatFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		# create column2Sizer widgets
		lowerLimitLabel = wx.StaticText(self, wx.ID_ANY, '     Lower Limit:')
		self.lowerLimitFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		upperLimitLabel = wx.StaticText(self, wx.ID_ANY, '     Upper Limit:')
		self.upperLimitFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		motor0FailedLabel = wx.StaticText(self, wx.ID_ANY, 'Motor 0 Failed:')
		self.motor0FailedFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)
	
		motor1FailedLabel = wx.StaticText(self, wx.ID_ANY, 'Motor 1 Failed:')
		self.motor1FailedFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		overMaxVelLabel = wx.StaticText(self, wx.ID_ANY, '   Over Max Vel:')
		self.overMaxVelFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		overMaxAccLabel = wx.StaticText(self, wx.ID_ANY, '  Over Max Acc:')
		self.overMaxAccFlag = wx.StaticBitmap(self, wx.ID_ANY, emptyImage)

		# Create sizers
		#
		# Create main BoxSizer for panel
		mainSizer = wx.BoxSizer(wx.VERTICAL)
		subSizer = wx.BoxSizer(wx.HORIZONTAL)
		column1Sizer = wx.BoxSizer(wx.VERTICAL)
		column2Sizer = wx.BoxSizer(wx.VERTICAL)

		# Create box sizers for section
		motor0FaultSizer = wx.BoxSizer(wx.HORIZONTAL)
		motor1FaultSizer = wx.BoxSizer(wx.HORIZONTAL)
		encoderFlagSizer = wx.BoxSizer(wx.HORIZONTAL)
		bigInputSizer   = wx.BoxSizer(wx.HORIZONTAL)
		motorJamSizer   = wx.BoxSizer(wx.HORIZONTAL)
		pwmSatSizer   = wx.BoxSizer(wx.HORIZONTAL)

		lowerLimitSizer = wx.BoxSizer(wx.HORIZONTAL)
		upperLimitSizer = wx.BoxSizer(wx.HORIZONTAL)
		motor0FailedSizer = wx.BoxSizer(wx.HORIZONTAL)
		motor1FailedSizer   = wx.BoxSizer(wx.HORIZONTAL)
		overMaxVelSizer   = wx.BoxSizer(wx.HORIZONTAL)
		overMaxAccSizer   = wx.BoxSizer(wx.HORIZONTAL)

		# Add widgets to respective sizers
		#
		# Add widgets to motor0FaultSizer
		motor0FaultSizer.Add(motor0FaultLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		motor0FaultSizer.Add(self.motor0FaultFlag)

		# Add widgets to motor1FaultSizer
		motor1FaultSizer.Add(motor1FaultLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		motor1FaultSizer.Add(self.motor1FaultFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to encoderFlagSizer
		encoderFlagSizer.Add(encoderLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		encoderFlagSizer.Add(self.encoderFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to bigInputSizer
		bigInputSizer.Add(bigInputLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		bigInputSizer.Add(self.bigInputFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to motorJamSizer
		motorJamSizer.Add(motorJamLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		motorJamSizer.Add(self.motorJamFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to pwmSatSizer
		pwmSatSizer.Add(pwmSatLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		pwmSatSizer.Add(self.pwmSatFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to lowerLimitSizer
		lowerLimitSizer.Add(lowerLimitLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lowerLimitSizer.Add(self.lowerLimitFlag)

		# Add widgets to upperLimitSizer
		upperLimitSizer.Add(upperLimitLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		upperLimitSizer.Add(self.upperLimitFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to motor0FailedSizer
		motor0FailedSizer.Add(motor0FailedLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		motor0FailedSizer.Add(self.motor0FailedFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to motor1FailedSizer
		motor1FailedSizer.Add(motor1FailedLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		motor1FailedSizer.Add(self.motor1FailedFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to overMaxVelSizer
		overMaxVelSizer.Add(overMaxVelLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		overMaxVelSizer.Add(self.overMaxVelFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add widgets to overMaxAccSizer
		overMaxAccSizer.Add(overMaxAccLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		overMaxAccSizer.Add(self.overMaxAccFlag, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add sizers to respective column sizers
		column1Sizer.Add(motor0FaultSizer, 0, wx.ALL|wx.EXPAND, 3)
		column1Sizer.Add(motor1FaultSizer, 0, wx.ALL|wx.EXPAND, 3)
		column1Sizer.Add(encoderFlagSizer, 0, wx.ALL|wx.EXPAND, 3)
		column1Sizer.Add(bigInputSizer, 0, wx.ALL|wx.EXPAND, 3)
		column1Sizer.Add(motorJamSizer, 0, wx.ALL|wx.EXPAND, 3)
		column1Sizer.Add(pwmSatSizer, 0, wx.ALL|wx.EXPAND, 3)

		column2Sizer.Add(lowerLimitSizer, 0, wx.ALL|wx.EXPAND, 3)
		column2Sizer.Add(upperLimitSizer, 0, wx.ALL|wx.EXPAND, 3)
		column2Sizer.Add(motor0FailedSizer, 0, wx.ALL|wx.EXPAND, 3)
		column2Sizer.Add(motor1FailedSizer, 0, wx.ALL|wx.EXPAND, 3)
		column2Sizer.Add(overMaxVelSizer, 0, wx.ALL|wx.EXPAND, 3)
		column2Sizer.Add(overMaxAccSizer, 0, wx.ALL|wx.EXPAND, 3)

		subSizer.Add(column1Sizer, 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(wx.StaticLine(self, wx.ID_ANY, size=(3,100), style=wx.LI_VERTICAL), 0, wx.ALL|wx.EXPAND, 3)
		subSizer.Add(column2Sizer, 0, wx.ALL|wx.EXPAND, 3)
	
		mainSizer.Add(panelLabel, 0, wx.CENTER, 3)
		mainSizer.Add(subSizer, 0, wx.ALL|wx.EXPAND, 3)		
		# Set sizer for panel
		self.SetSizer(mainSizer)


