#!/usr/bin/python
import wx				# wxWidgets library

# Set wx.TextCtrl widget (text box) dimensions
widgetHeight = 25
textBoxWidth = 50
comboBoxWidth = 110
spinCtrlWidth = 80
toggleButtonWidth = 70
radioBoxWidth = 100


class Settings_Frame(wx.Frame):
	def __init__(self, *args, **kwargs):
		wx.Frame.__init__(self, *args, **kwargs)
		self.CreateStatusBar() # put status bar at bottom of window

		# Set up the menu at the top
		filemenu = wx.Menu() # create Menu widget
		# add About selection to the file menu with (Id, text, status bar text) parameters
		menuAbout = filemenu.Append(wx.ID_ABOUT, "&About", "Information about this program") 
		# add Exit selection to the file menu with (Id, text, status bar text) parameters
		menuExit = filemenu.Append(wx.ID_EXIT, "&Exit", "Terminate the program") 

		# Set up edit menu
		editmenu = wx.Menu()
		menuCopy = editmenu.Append(wx.ID_COPY, "&Copy", "Copy")

		# Create the menu bar at the top
		menuBar = wx.MenuBar() # create a MenuBar widget
		menuBar.Append(filemenu, "&File") # add file menu "File" to menu bar
		menuBar.Append(editmenu, "&Edit") # add edit menu "Edit" to menu bar
		self.SetMenuBar(menuBar) # set the menu bar for the frame to menuBar

		# Create main BoxSizer and put all the panels into it
		self.settingsPanel = Settings_Panel(self)
		self.boardSettingsPanel = Board_Settings(self)
		self.deadZonePanel = Dead_Zone(self)

		# Set events
		self.Bind(wx.EVT_MENU, self.OnAbout, menuAbout) # bind menuAbout object to select menu item event, and call the function "OnAbout"
		self.Bind(wx.EVT_MENU, self.OnExit, menuExit) # bind menuExit object to select menu item event, and call the function "OnExit"
		self.Bind(wx.EVT_MENU, self.OnCopy, menuCopy) # bind menuCopy object to select menu item event, and call the function "OnCopy"
		self.Bind(wx.EVT_LISTBOX, self.OnListBox, self.settingsPanel.settingsList)
		self.Bind(wx.EVT_BUTTON, self.OnEditButton, self.boardSettingsPanel.boardEditButton)

		frameSize = self.GetSize()
		# Create main sizer and add all panels to it
		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		self.subSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.subSizer.Add(self.settingsPanel, 0, border=5)
	#	self.subSizer.Add(self.boardSettingsPanel, 0, wx.RESERVE_SPACE_EVEN_IF_HIDDEN, border=5)
	#	self.subSizer.Add(self.deadZonePanel, 0, wx.RESERVE_SPACE_EVEN_IF_HIDDEN, border=5)
	#	self.subSizer.Hide(self.boardSettingsPanel, True)
	#	self.subSizer.Hide(self.deadZonePanel, True)
		self.mainSizer.Add(self.subSizer, 0, wx.ALL|wx.EXPAND, border=5)
		self.SetSizer(self.mainSizer)
		self.Show()


	# A message dialog box with an OK button
	def OnAbout(self, event):
		dlg = wx.MessageDialog(self, "Feedback\n1.0\n\nA program to display feedback for all of Hubo's sensors and motors.", "About Feedback", wx.OK) # create MessageDialog widget with (Text to be displayed, frame title, Id) parameters. wx.OK is standard ID in wxWidgets
		dlg.ShowModal() # display dialog box
		dlg.Destroy() # destory it when finished

	# Close the program with "Exit" is selected from the File menu
	def OnExit(self, event):
		self.Close(True) # close the frame

	# Copy highlighted object
	def OnCopy(self, event):
		self.dataObj = wx.TextDataObject()
		self.dataObj.SetText(self.FindFocus().GetStringSelection())
		if wx.TheClipboard.Open():
			wx.TheClipboard.SetData(self.dataObj)
			wx.TheClipboard.Flush()
		else:
			wx.MessageBox("Unable to open clipboard", "Error")
			wx.TheClipboard.Close()

	def OnListBox(self, event):
		indexSelected = self.settingsPanel.settingsList.GetSelection()
		selection = self.settingsPanel.settingsList.GetString(indexSelected)

		self.subSizer.Hide(1)
		self.subSizer.Remove(1)

		if selection == 'Board Settings':
			self.subSizer.Add(self.boardSettingsPanel, 0, border=5)
			self.subSizer.Show(1)
			self.mainSizer.Layout()

		if selection == 'Dead Zone':
			self.subSizer.Add(self.deadZonePanel, 0, border=5)
			self.subSizer.Show(1)
			self.mainSizer.Layout()

	def OnEditButton(self, event):
		self.boardSettingsPanel.boardNumValue.SetEditable(True)


class Settings_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		settingsOptions = ['Board Settings', 'Board Errors', 'Current Limit', 'Dead Zone', 'Encoder Res.', 
							'Error Limit', 'Gain Scale',  'Home Params',  'Home Vel/Acc', 'Jam Sat. Limit', 
							'JMC Alarm', 'JMC Beep', 'Lower Pos. Limit', 'Open Loop PWM', 'PID Gain',
						    'Set Current Gain', 'Set Pos. Gain', 'Upper Pos. Limit', 'Vmax/Amax']


		# Create main BoxSizer and put all the panels into it
		self.mainPanel = wx.Panel(self)

		# Create settings panel widgets
		self.settingsList = wx.ListBox(self.mainPanel, wx.ID_ANY, size=wx.Size(200, len(settingsOptions)*25), choices=settingsOptions)

		# Create main sizer and add all panels to it
		self.mainSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.column1Sizer = wx.BoxSizer(wx.VERTICAL)
		self.column2Sizer = wx.BoxSizer(wx.VERTICAL)

		# Add sizers to main panel
		self.column1Sizer.Add(wx.StaticText(self.mainPanel, wx.ID_ANY, 'Settings Menu'), 0, wx.CENTER, 3)
		self.column1Sizer.Add(self.settingsList, 0, border=5)
		self.mainSizer.Add(self.column1Sizer, 0, border=5)
		self.mainSizer.Add(self.column2Sizer, 0, border=5)
		self.mainPanel.SetSizerAndFit(self.mainSizer)


class Board_Settings(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		self.mainPanel = wx.Panel(self)

		# Create widgets
		paramLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, 'Parameters for ')
		self.boardLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, '')

		boardNumLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, '            Board Number: ')
		self.boardNumValue = wx.TextCtrl(self.mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		self.boardEditButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Edit', size=wx.Size(textBoxWidth, widgetHeight)) 
		self.boardSetButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Set', size=wx.Size(textBoxWidth, widgetHeight)) 

		canRateLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, '               CAN Rate(Hz): ')
		self.canRateValue = wx.TextCtrl(self.mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		self.canRateEditButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Edit', size=wx.Size(textBoxWidth, widgetHeight)) 
		self.canRateSetButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Set', size=wx.Size(textBoxWidth, widgetHeight)) 

		numOfChanLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, 'Number of Channels: ')
		self.numOfChanValue = wx.TextCtrl(self.mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		self.chanEditButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Edit', size=wx.Size(textBoxWidth, widgetHeight)) 
		self.chanSetButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Set', size=wx.Size(textBoxWidth, widgetHeight)) 


		self.mainSizer = wx.BoxSizer(wx.VERTICAL)

		self.oneSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.twoSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.threeSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.fourSizer = wx.BoxSizer(wx.HORIZONTAL)
		

		self.oneSizer.Add(paramLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.oneSizer.Add(self.boardLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		self.twoSizer.Add(boardNumLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.boardNumValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.boardEditButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.boardSetButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		self.threeSizer.Add(canRateLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.threeSizer.Add(self.canRateValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.threeSizer.Add(self.canRateEditButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.threeSizer.Add(self.canRateSetButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		self.fourSizer.Add(numOfChanLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.fourSizer.Add(self.numOfChanValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.fourSizer.Add(self.chanEditButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.fourSizer.Add(self.chanSetButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)


		self.mainSizer.Add(self.oneSizer, 0, wx.ALIGN_CENTER, 3)
		self.mainSizer.Add(self.twoSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.threeSizer, 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.fourSizer, 0, wx.ALL|wx.EXPAND, 3)

		self.mainPanel.SetSizerAndFit(self.mainSizer)


class Dead_Zone(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		self.mainPanel = wx.Panel(self)

		# Create widgets
		paramLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, 'Parameters for ')
		self.boardLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, '')

		deadZoneLabel = wx.StaticText(self.mainPanel, wx.ID_ANY, 'Dead Zone: ')
		self.deadZoneValue = wx.TextCtrl(self.mainPanel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))
		self.deadZoneEditButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Edit', size=wx.Size(textBoxWidth, widgetHeight)) 
		self.deadZoneSetButton = wx.Button(self.mainPanel, wx.ID_ANY, 'Set', size=wx.Size(textBoxWidth, widgetHeight)) 


		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		self.oneSizer = wx.BoxSizer(wx.HORIZONTAL)	
		self.twoSizer = wx.BoxSizer(wx.HORIZONTAL)	


		self.oneSizer.Add(paramLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.oneSizer.Add(self.boardLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		self.twoSizer.Add(deadZoneLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.deadZoneValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.deadZoneEditButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		self.twoSizer.Add(self.deadZoneSetButton, 0, wx.ALL|wx.ALIGN_CENTER, 1)


		self.mainSizer.Add(self.oneSizer, 0, wx.ALIGN_CENTER, 3)
		self.mainSizer.Add(self.twoSizer, 0, wx.ALIGN_CENTER, 3)

		self.mainPanel.SetSizerAndFit(self.mainSizer)



if __name__ == '__main__':
	app = wx.App(False) # create a new app. False means don't redirect stdout/stderr to new window
	frame = Settings_Frame(None, wx.ID_ANY, pos=wx.Point(0,0), title='Settings', size=wx.Size(400,len(Settings_Panel.settingsOptions)*225)) # frame constructor. Frame(parent, Id, title)
#	frame.Fit()
	frame.Show() # show frame
	app.MainLoop() # start app's main loop, handling events
