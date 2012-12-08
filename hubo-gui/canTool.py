#!/usr/bin/python
import wx				# wxWidgets library
import jointsPanel 		# Module which creates a panel of all joints with textctrls for each
import motorPanel	# Module which creates a single force/torque panel

class Hubo_CanTool(wx.Frame):
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

		# Create zeroed status panel
		self.jointsPanel = jointsPanel.Joints_Panel(mainPanel, name='Joint Selection')
		self.motorPanel = motorPanel.Motor_Panel(mainPanel,)
		# Create force/torque panels with names according location of the force/torque sensors
#		self.leftWristFTPanel = motorPanel.Motor_Panel(mainPanel, name='Control')

		# Create force/torque BoxSizer and add panels
#		self.motorPanelSizer = wx.BoxSizer(wx.HORIZONTAL)
#		self.motorPanelSizer.Add(self.leftWristFTPanel, 0, border=5)

		# Create main sizer and add all panels to it
		self.mainSizer = wx.BoxSizer(wx.VERTICAL)
		self.mainSizer.Add(self.jointsPanel, 0, wx.CENTER, border=5)
		self.mainSizer.Add(wx.StaticLine(mainPanel,), 0, wx.ALL|wx.EXPAND, 3)
		self.mainSizer.Add(self.motorPanel, 0, border=5)
		self.mainSizer.Add(wx.StaticLine(mainPanel,), 0, wx.ALL|wx.EXPAND, 3)
		mainPanel.SetSizerAndFit(self.mainSizer)
		self.Show()


		self.errorImage = wx.Image('error-icon.png', wx.BITMAP_TYPE_ANY, -1)
		self.errorImage = wx.BitmapFromImage(self.errorImage.Scale(20,20, quality=(wx.IMAGE_QUALITY_HIGH)))
		self.emptyImage = wx.EmptyBitmap(0,0)
		

	# A message dialog box with an OK button
	def OnAbout(self, event):
		self.motorPanel.errorsPanel.bigInputFlag.SetBitmap(self.errorImage)
		self.motorPanel.errorsPanel.encoderFlag.SetBitmap(self.errorImage)
		self.motorPanel.boardNumValue.SetValue('36')
		self.motorPanel.boardChanValue.SetValue('2')
		self.motorPanel.jointValue.SetValue('WST')
		dlg = wx.MessageDialog(self, "Hubo CAN Tool\n1.0\n\nA CAN Tool program to interact with and configure all of Hubo's joint motor controllers (JMC). This tool communicates over Hubo's CAN (Controller Area Network) channels.", "About Hubo CAN Tool", wx.OK) # create MessageDialog widget with (Text to be displayed, frame title, Id) parameters. wx.OK is standard ID in wxWidgets
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


if __name__ == '__main__':
	app = wx.App(False) # create a new app. False means don't redirect stdout/stderr to new window
	frame = Hubo_CanTool(None, wx.ID_ANY, title='Hubo CAN Tool') # frame constructor. Frame(parent, Id, title)
	frame.Fit()
	frame.Show() # show frame
	app.MainLoop() # start app's main loop, handling events
