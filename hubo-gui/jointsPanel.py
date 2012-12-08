#!/usr/bin/python
import wx

class Joints_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Get panel name from kwargs parameter in order to put a title at the top of the panel
		for keyword in kwargs:
			if keyword == 'name':
				panelName = kwargs[keyword]

		# Set wx.TextCtrl (text box) dimensions
		jointButtonWidth  = 50
		sensorButtonWidth = 130
		widgetHeight = 30	

		# Define widgets for all joints		
		self.wstButton = wx.Button(self, wx.ID_ANY, 'WST', size=wx.Size(jointButtonWidth,widgetHeight))
		self.nkyButton = wx.Button(self, wx.ID_ANY, 'NKY', size=wx.Size(jointButtonWidth,widgetHeight))
		self.nk1Button = wx.Button(self, wx.ID_ANY, 'NK1', size=wx.Size(jointButtonWidth,widgetHeight))
		self.nk2Button = wx.Button(self, wx.ID_ANY, 'NK2', size=wx.Size(jointButtonWidth,widgetHeight))

		self.lspButton = wx.Button(self, wx.ID_ANY, ' LSP ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lsrButton = wx.Button(self, wx.ID_ANY, ' LSR ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lsyButton = wx.Button(self, wx.ID_ANY, ' LSY ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lebButton = wx.Button(self, wx.ID_ANY, ' LEB ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lwyButton = wx.Button(self, wx.ID_ANY, 'LWY', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lwrButton = wx.Button(self, wx.ID_ANY, 'LWR', size=wx.Size(jointButtonWidth,widgetHeight))

		self.rspButton = wx.Button(self, wx.ID_ANY, ' RSP ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rsrButton = wx.Button(self, wx.ID_ANY, ' RSR ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rsyButton = wx.Button(self, wx.ID_ANY, ' RSY ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rebButton = wx.Button(self, wx.ID_ANY, ' REB ', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rwyButton = wx.Button(self, wx.ID_ANY, 'RWY', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rwrButton = wx.Button(self, wx.ID_ANY, 'RWR', size=wx.Size(jointButtonWidth,widgetHeight))

		self.lhyButton = wx.Button(self, wx.ID_ANY, 'LHY', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lhrButton = wx.Button(self, wx.ID_ANY, 'LHR', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lhpButton = wx.Button(self, wx.ID_ANY, 'LHP', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lknButton = wx.Button(self, wx.ID_ANY, 'LKN', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lapButton = wx.Button(self, wx.ID_ANY, 'LAP', size=wx.Size(jointButtonWidth,widgetHeight))
		self.larButton = wx.Button(self, wx.ID_ANY, 'LAR', size=wx.Size(jointButtonWidth,widgetHeight))

		self.rhyButton = wx.Button(self, wx.ID_ANY, 'RHY', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rhrButton = wx.Button(self, wx.ID_ANY, 'RHR', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rhpButton = wx.Button(self, wx.ID_ANY, 'RHP', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rknButton = wx.Button(self, wx.ID_ANY, 'RKN', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rapButton = wx.Button(self, wx.ID_ANY, 'RAP', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rarButton = wx.Button(self, wx.ID_ANY, 'RAR', size=wx.Size(jointButtonWidth,widgetHeight))

		self.rf1Button = wx.Button(self, wx.ID_ANY, 'RF1', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rf2Button = wx.Button(self, wx.ID_ANY, 'RF2', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rf3Button = wx.Button(self, wx.ID_ANY, 'RF3', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rf4Button = wx.Button(self, wx.ID_ANY, 'RF4', size=wx.Size(jointButtonWidth,widgetHeight))
		self.rf5Button = wx.Button(self, wx.ID_ANY, 'RF5', size=wx.Size(jointButtonWidth,widgetHeight))

		self.lf1Button = wx.Button(self, wx.ID_ANY, 'LF1', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lf2Button = wx.Button(self, wx.ID_ANY, 'LF2', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lf3Button = wx.Button(self, wx.ID_ANY, 'LF3', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lf4Button = wx.Button(self, wx.ID_ANY, 'LF4', size=wx.Size(jointButtonWidth,widgetHeight))
		self.lf5Button = wx.Button(self, wx.ID_ANY, 'LF5', size=wx.Size(jointButtonWidth,widgetHeight))

		self.ftLeftWristButton = wx.Button(self, wx.ID_ANY, 'F/T Left Ankle', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.ftRightWristButton = wx.Button(self, wx.ID_ANY, 'F/T Right Ankle', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.ftLeftAnkleButton = wx.Button(self, wx.ID_ANY, 'F/T Left Foot', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.ftRightAnkleButton = wx.Button(self, wx.ID_ANY, 'F/T Right Foot', size=wx.Size(sensorButtonWidth,widgetHeight))

		self.imuWaistButton = wx.Button(self, wx.ID_ANY, 'IMU Waist', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.imuLeftFootButton = wx.Button(self, wx.ID_ANY, 'IMU Left Foot', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.imuRightFootButton = wx.Button(self, wx.ID_ANY, 'IMU Right Foot', size=wx.Size(sensorButtonWidth,widgetHeight))
		self.smartPowerButton = wx.Button(self, wx.ID_ANY, 'Smart Power Ctrl', size=wx.Size(sensorButtonWidth,widgetHeight))

		# Sizer for whole frame
	   	self.mainSizer = wx.BoxSizer(wx.HORIZONTAL)

		# Sizers for panel sections selection
		self.jointSelectionSizer = wx.BoxSizer(wx.VERTICAL)

		# Sizer for joints
		self.jointsSizer = wx.BoxSizer(wx.HORIZONTAL)

		# 

		# Sizers for each column of joints
		self.column1Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column2Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column3Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column4Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column5Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column6Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column7Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column8Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column9Sizer = wx.BoxSizer(wx.VERTICAL)


		# Add each joint sizer to its appropriate column
		self.column1Sizer.Add(self.wstButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column1Sizer.Add(self.nkyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column1Sizer.Add(self.nk1Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column1Sizer.Add(self.nk2Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column2Sizer.Add(self.lspButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column2Sizer.Add(self.lsyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column2Sizer.Add(self.lsrButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column2Sizer.Add(self.lebButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column2Sizer.Add(self.lwyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column2Sizer.Add(self.lwrButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column3Sizer.Add(self.rspButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column3Sizer.Add(self.rsrButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column3Sizer.Add(self.rsyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column3Sizer.Add(self.rebButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column3Sizer.Add(self.rwyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column3Sizer.Add(self.rwrButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column4Sizer.Add(self.lhyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column4Sizer.Add(self.lhrButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column4Sizer.Add(self.lhpButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column4Sizer.Add(self.lknButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column4Sizer.Add(self.lapButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column4Sizer.Add(self.larButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column5Sizer.Add(self.rhyButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column5Sizer.Add(self.rhrButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column5Sizer.Add(self.rhpButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column5Sizer.Add(self.rknButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column5Sizer.Add(self.rapButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column5Sizer.Add(self.rarButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column6Sizer.Add(self.rf1Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column6Sizer.Add(self.rf2Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column6Sizer.Add(self.rf3Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column6Sizer.Add(self.rf4Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column6Sizer.Add(self.rf5Button, 0, wx.ALL|wx.EXPAND, 2)

		self.column7Sizer.Add(self.lf1Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column7Sizer.Add(self.lf2Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column7Sizer.Add(self.lf3Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column7Sizer.Add(self.lf4Button, 0, wx.ALL|wx.EXPAND, 2)
		self.column7Sizer.Add(self.lf5Button, 0, wx.ALL|wx.EXPAND, 2)

		self.column8Sizer.Add(self.ftLeftWristButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column8Sizer.Add(self.ftRightWristButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column8Sizer.Add(self.ftLeftAnkleButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column8Sizer.Add(self.ftRightAnkleButton, 0, wx.ALL|wx.EXPAND, 2)

		self.column9Sizer.Add(self.imuWaistButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column9Sizer.Add(self.imuLeftFootButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column9Sizer.Add(self.imuRightFootButton, 0, wx.ALL|wx.EXPAND, 2)
		self.column9Sizer.Add(self.smartPowerButton, 0, wx.ALL|wx.EXPAND, 2)

		# Add the column sizers to the joints sizer
		self.jointsSizer.Add(self.column1Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column2Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column3Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column4Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column5Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column6Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column7Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column8Sizer, 0, wx.ALL|wx.EXPAND, 3)
		self.jointsSizer.Add(self.column9Sizer, 0, wx.ALL|wx.EXPAND, 3)

		self.jointSelectionSizer.Add(wx.StaticText(self, wx.ID_ANY, 'Joint/Sensor Selection'), 0, wx.CENTER, 3)
		self.jointSelectionSizer.Add(self.jointsSizer, 0, wx.ALL|wx.EXPAND, 3)

		# Add all sections to main sizer
		self.mainSizer.Add(wx.StaticLine(self,), 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.jointSelectionSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.SetSizer(self.mainSizer)
