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

		# Set events
		self.Bind(wx.EVT_MENU, self.OnAbout, menuAbout) # bind menuAbout object to select menu item event, and call the function "OnAbout"
		self.Bind(wx.EVT_MENU, self.OnExit, menuExit) # bind menuExit object to select menu item event, and call the function "OnExit"
		self.Bind(wx.EVT_MENU, self.OnCopy, menuCopy) # bind menuCopy object to select menu item event, and call the function "OnCopy"

		# Create main BoxSizer and put all the panels into it
		mainPanel = wx.Panel(self)
		self.settingsPanel = Settings_Panel(mainPanel)

		# Create main sizer and add all panels to it
		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		self.mainSizer.Add(wx.StaticText(self, wx.ID_ANY, 'Settings Menu'), 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.settingsPanel, 0, border=5)
		mainPanel.SetSizerAndFit(self.mainSizer)
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


class Settings_Panel(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		settingsOptions = ['Board Settings', 'Board Errors', 'Current Limit', 'Dead Zone', 'Encoder Res.', 
							'Error Limit', 'Gain Scale',  'Home Params',  'Home Vel/Acc', 'Jam Sat. Limit', 
							'JMC Alarm', 'JMC Beep', 'Lower Pos. Limit', 'Open Loop PWM', 'PID Gain',
						    'Set Current Gain', 'Set Pos. Gain', 'Upper Pos. Limit', 'Vmax/Amax']


		# Create main BoxSizer and put all the panels into it
		self.mainPanel = wx.Panel(self)
		boardSettingsPanel = Board_Settings(self.mainPanel)

		# Create settings panel widgets
		self.settingsList = wx.ListBox(self.mainPanel, wx.ID_ANY, size=wx.Size(200, len(settingsOptions)*25), choices=settingsOptions)

		# Set event bindings to objects
		self.Bind(wx.EVT_LISTBOX, self.OnListBox, self.settingsList)

		# Create main sizer and add all panels to it
		self.mainSizer = wx.BoxSizer(wx.HORIZONTAL)

		# Add sizers to main panel
		self.mainSizer.Add(self.settingsList, 0, border=5)
		self.mainPanel.SetSizerAndFit(self.mainSizer)

	def OnListBox(self, event):
		indexSelected = self.settingsList.GetSelection()
		selection = self.settingsList.GetString(indexSelected)

		if selection == 'Board Settings':
			self.mainSizer.Add(self.settingsList, 0, border=5)
			self.mainPanel.SetSizerAndFit(self.mainSizer)	
#		elif selection == settingsOptions(1):
			
#		elif selection == settingsOptions(2):
			
#		elif selection == settingsOptions(3):
			
#		elif selection == settingsOptions(4):
			
#		elif selection == settingsOptions(5):
			
#		elif selection == settingsOptions(6):
			
#		elif selection == settingsOptions(7):
			
#		elif selection == settingsOptions(8):
			
#		elif selection == settingsOptions(9):
			
#		elif selection == settingsOptions(10):
			
#		elif selection == settingsOptions(11):
			
#		elif selection == settingsOptions(12):
			
#		elif selection == settingsOptions(13):
			
#		elif selection == settingsOptions(14):
			
#		elif selection == settingsOptions(15):
			
#		elif selection == settingsOptions(16):
			
#		elif selection == settingsOptions(17):
			
#		elif selection == settingsOptions(18):
			
#		elif selection == settingsOptions(19):
			

class Board_Settings(wx.Panel):
	def __init__(self, *args, **kwargs):
		wx.Panel.__init__(self, *args, **kwargs)

		# Create widgets
		paramLabel = wx.StaticText(self, wx.ID_ANY, 'Parameters for ')
		self.boardLabel = wx.StaticText(self, wx.ID_ANY, '')

		boardNumLabel = wx.StaticText(self, wx.ID_ANY, 'Board Number: ')
		self.boardNumValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))

		canRateLabel = wx.StaticText(self, wx.ID_ANY, 'CAN Rate: ')
		self.canRateValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))

		numOfChanLabel = wx.StaticText(self, wx.ID_ANY, 'Number of Channels: ')
		self.numOfChanValue = wx.TextCtrl(self, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textBoxWidth, widgetHeight))


		mainSizer = wx.BoxSizer(wx.VERTICAL)

		paramSizer = wx.BoxSizer(wx.HORIZONTAL)
		boardSizer = wx.BoxSizer(wx.HORIZONTAL)
		canSizer = wx.BoxSizer(wx.HORIZONTAL)
		chanSizer = wx.BoxSizer(wx.HORIZONTAL)
		

		paramSizer.Add(paramLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		paramSizer.Add(self.boardLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		boardSizer.Add(boardNumLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		boardSizer.Add(self.boardNumValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		canSizer.Add(canRateLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		canSizer.Add(self.canRateValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		chanSizer.Add(numOfChanLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		chanSizer.Add(self.numOfChanValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)


		mainSizer.Add(paramSizer, 0, wx.ALIGN_CENTER, 3)
		mainSizer.Add(boardSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(canSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(chanSizer, 0, wx.ALL|wx.EXPAND, 3)


		self.SetSizerAndFit(mainSizer)

if __name__ == '__main__':
	app = wx.App(False) # create a new app. False means don't redirect stdout/stderr to new window
	frame = Settings_Frame(None, wx.ID_ANY, title='Settings') # frame constructor. Frame(parent, Id, title)
	frame.Fit()
	frame.Show() # show frame
	app.MainLoop() # start app's main loop, handling events
