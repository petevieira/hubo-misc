#!/usr/bin/python
import wx

class Feedback(wx.Frame):
	def __init__(self):
		wx.Frame.__init__(self, None, wx.ID_ANY, title='Feedback')
		self.CreateStatusBar() # put status bar at bottom of window

		# Set up the menu at the top
		filemenu = wx.Menu() # create Menu widget
		menuAbout = filemenu.Append(wx.ID_ABOUT, "&About", "Information about this program") # add About selection to the file menu with (Id, text, status bar text) parameters
		#filemenu.AppendSeparator()	# add separator line
		menuExit = filemenu.Append(wx.ID_EXIT, "&Exit", "Terminate the program") # add Exit selection to the file menu with (Id, text, status bar text) parameters

		# Create the menu bar at the top
		menuBar = wx.MenuBar() # create a MenuBar widget
		menuBar.Append(filemenu, "&File") # add file menu "File" to menu bar
		self.SetMenuBar(menuBar) # set the menu bar for the frame to menuBar

		panel = wx.Panel(self, wx.ID_ANY)

		textWidth = 50
		textHeight = 25	
		# Define widgets for all joints		
		wstLabel = wx.StaticText(panel, wx.ID_ANY, 'WST')
		wstValue = wx.TextCtrl(panel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		nkyLabel = wx.StaticText(panel, wx.ID_ANY, 'NKY')
		nkyValue = wx.TextCtrl(panel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		nk1Label = wx.StaticText(panel, wx.ID_ANY, 'NK1')
		nk1Value = wx.TextCtrl(panel, wx.ID_ANY, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		nk2Label = wx.StaticText(panel, -1, 'NK2')
		nk2Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		lspLabel = wx.StaticText(panel, -1, ' LSP ')
		lspValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lsrLabel = wx.StaticText(panel, -1, ' LSR ')
		lsrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lsyLabel = wx.StaticText(panel, -1, ' LSY ')
		lsyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lebLabel = wx.StaticText(panel, -1, ' LEB ')
		lebValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lwyLabel = wx.StaticText(panel, -1, 'LWY')
		lwyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lwrLabel = wx.StaticText(panel, -1, 'LWR')
		lwrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		rspLabel = wx.StaticText(panel, -1, ' RSP ')
		rspValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rsrLabel = wx.StaticText(panel, -1, ' RSR ')
		rsrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rsyLabel = wx.StaticText(panel, -1, ' RSY ')
		rsyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rebLabel = wx.StaticText(panel, -1, ' REB ')
		rebValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rwyLabel = wx.StaticText(panel, -1, 'RWY')
		rwyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rwrLabel = wx.StaticText(panel, -1, 'RWR')
		rwrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		lhyLabel = wx.StaticText(panel, -1, 'LHY')
		lhyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lhrLabel = wx.StaticText(panel, -1, 'LHR')
		lhrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lhpLabel = wx.StaticText(panel, -1, 'LHP')
		lhpValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lknLabel = wx.StaticText(panel, -1, 'LKN')
		lknValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lapLabel = wx.StaticText(panel, -1, 'LAP')
		lapValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		larLabel = wx.StaticText(panel, -1, 'LAR')
		larValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		rhyLabel = wx.StaticText(panel, -1, 'RHY')
		rhyValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rhrLabel = wx.StaticText(panel, -1, 'RHR')
		rhrValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rhpLabel = wx.StaticText(panel, -1, 'RHP')
		rhpValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rknLabel = wx.StaticText(panel, -1, 'RKN')
		rknValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rapLabel = wx.StaticText(panel, -1, 'RAP')
		rapValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rarLabel = wx.StaticText(panel, -1, 'RAR')
		rarValue = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		rf1Label = wx.StaticText(panel, -1, 'RF1')
		rf1Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rf2Label = wx.StaticText(panel, -1, 'RF2')
		rf2Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rf3Label = wx.StaticText(panel, -1, 'RF3')
		rf3Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rf4Label = wx.StaticText(panel, -1, 'RF4')
		rf4Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		rf5Label = wx.StaticText(panel, -1, 'RF5')
		rf5Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		lf1Label = wx.StaticText(panel, -1, 'LF1')
		lf1Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lf2Label = wx.StaticText(panel, -1, 'LF2')
		lf2Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lf3Label = wx.StaticText(panel, -1, 'LF3')
		lf3Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lf4Label = wx.StaticText(panel, -1, 'LF4')
		lf4Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))
		lf5Label = wx.StaticText(panel, -1, 'LF5')
		lf5Value = wx.TextCtrl(panel, -1, '', style=wx.TE_READONLY, size=wx.Size(textWidth, textHeight))

		# Sizer for whole frame
		mainSizer = wx.BoxSizer(wx.VERTICAL)

		# Sizer for Zeroed section
		zeroedSizer = wx.BoxSizer(wx.HORIZONTAL)

		# Sizers for each column of joints
		column1Sizer = wx.BoxSizer(wx.VERTICAL)
		wstSizer = wx.BoxSizer(wx.HORIZONTAL)
		nkySizer = wx.BoxSizer(wx.HORIZONTAL)
		nk1Sizer = wx.BoxSizer(wx.HORIZONTAL)
		nk2Sizer = wx.BoxSizer(wx.HORIZONTAL)

		column2Sizer = wx.BoxSizer(wx.VERTICAL)
		lspSizer = wx.BoxSizer(wx.HORIZONTAL)
		lsrSizer = wx.BoxSizer(wx.HORIZONTAL)
		lsySizer = wx.BoxSizer(wx.HORIZONTAL)
		lebSizer = wx.BoxSizer(wx.HORIZONTAL)
		lwySizer = wx.BoxSizer(wx.HORIZONTAL)
		lwrSizer = wx.BoxSizer(wx.HORIZONTAL)

		column3Sizer = wx.BoxSizer(wx.VERTICAL)
		rspSizer = wx.BoxSizer(wx.HORIZONTAL)
		rsrSizer = wx.BoxSizer(wx.HORIZONTAL)
		rsySizer = wx.BoxSizer(wx.HORIZONTAL)
		rebSizer = wx.BoxSizer(wx.HORIZONTAL)
		rwySizer = wx.BoxSizer(wx.HORIZONTAL)
		rwrSizer = wx.BoxSizer(wx.HORIZONTAL)

		column4Sizer = wx.BoxSizer(wx.VERTICAL)
		lhySizer = wx.BoxSizer(wx.HORIZONTAL)
		lhrSizer = wx.BoxSizer(wx.HORIZONTAL)
		lhpSizer = wx.BoxSizer(wx.HORIZONTAL)
		lknSizer = wx.BoxSizer(wx.HORIZONTAL)
		lapSizer = wx.BoxSizer(wx.HORIZONTAL)
		larSizer = wx.BoxSizer(wx.HORIZONTAL)
		
		column5Sizer = wx.BoxSizer(wx.VERTICAL)
		rhySizer = wx.BoxSizer(wx.HORIZONTAL)
		rhrSizer = wx.BoxSizer(wx.HORIZONTAL)
		rhpSizer = wx.BoxSizer(wx.HORIZONTAL)
		rknSizer = wx.BoxSizer(wx.HORIZONTAL)
		rapSizer = wx.BoxSizer(wx.HORIZONTAL)
		rarSizer = wx.BoxSizer(wx.HORIZONTAL)

		column6Sizer = wx.BoxSizer(wx.VERTICAL)
		rf1Sizer = wx.BoxSizer(wx.HORIZONTAL)
		rf2Sizer = wx.BoxSizer(wx.HORIZONTAL)
		rf3Sizer = wx.BoxSizer(wx.HORIZONTAL)
		rf4Sizer = wx.BoxSizer(wx.HORIZONTAL)
		rf5Sizer = wx.BoxSizer(wx.HORIZONTAL)

		column7Sizer = wx.BoxSizer(wx.VERTICAL)
		lf1Sizer = wx.BoxSizer(wx.HORIZONTAL)
		lf2Sizer = wx.BoxSizer(wx.HORIZONTAL)
		lf3Sizer = wx.BoxSizer(wx.HORIZONTAL)
		lf4Sizer = wx.BoxSizer(wx.HORIZONTAL)
		lf5Sizer = wx.BoxSizer(wx.HORIZONTAL)

		# Add joint widgets to each joint sizer
		wstSizer.Add(wstLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		wstSizer.Add(wstValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nkySizer.Add(nkyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nkySizer.Add(nkyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nk1Sizer.Add(nk1Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nk1Sizer.Add(nk1Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nk2Sizer.Add(nk2Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		nk2Sizer.Add(nk2Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		lspSizer.Add(lspLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lspSizer.Add(lspValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lsrSizer.Add(lsrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lsrSizer.Add(lsrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lsySizer.Add(lsyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lsySizer.Add(lsyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lebSizer.Add(lebLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lebSizer.Add(lebValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lwySizer.Add(lwyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lwySizer.Add(lwyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lwrSizer.Add(lwrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lwrSizer.Add(lwrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		rspSizer.Add(rspLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rspSizer.Add(rspValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rsrSizer.Add(rsrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rsrSizer.Add(rsrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rsySizer.Add(rsyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rsySizer.Add(rsyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rebSizer.Add(rebLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rebSizer.Add(rebValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rwySizer.Add(rwyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rwySizer.Add(rwyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rwrSizer.Add(rwrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rwrSizer.Add(rwrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		lhySizer.Add(lhyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lhySizer.Add(lhyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lhrSizer.Add(lhrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lhrSizer.Add(lhrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lhpSizer.Add(lhpLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lhpSizer.Add(lhpValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lknSizer.Add(lknLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lknSizer.Add(lknValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lapSizer.Add(lapLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lapSizer.Add(lapValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		larSizer.Add(larLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		larSizer.Add(larValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		rhySizer.Add(rhyLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rhySizer.Add(rhyValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rhrSizer.Add(rhrLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rhrSizer.Add(rhrValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rhpSizer.Add(rhpLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rhpSizer.Add(rhpValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rknSizer.Add(rknLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rknSizer.Add(rknValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rapSizer.Add(rapLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rapSizer.Add(rapValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rarSizer.Add(rarLabel, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rarSizer.Add(rarValue, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		rf1Sizer.Add(rf1Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf1Sizer.Add(rf1Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf2Sizer.Add(rf2Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf2Sizer.Add(rf2Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf3Sizer.Add(rf3Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf3Sizer.Add(rf3Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf4Sizer.Add(rf4Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf4Sizer.Add(rf4Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf5Sizer.Add(rf5Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		rf5Sizer.Add(rf5Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		lf1Sizer.Add(lf1Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf1Sizer.Add(lf1Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf2Sizer.Add(lf2Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf2Sizer.Add(lf2Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf3Sizer.Add(lf3Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf3Sizer.Add(lf3Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf4Sizer.Add(lf4Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf4Sizer.Add(lf4Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf5Sizer.Add(lf5Label, 0, wx.ALL|wx.ALIGN_CENTER, 1)
		lf5Sizer.Add(lf5Value, 0, wx.ALL|wx.ALIGN_CENTER, 1)

		# Add each joint sizer to its appropriate column
		column1Sizer.Add(wstSizer, 0, wx.ALL|wx.EXPAND, 2)
		column1Sizer.Add(nkySizer, 0, wx.ALL|wx.EXPAND, 2)
		column1Sizer.Add(nk1Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column1Sizer.Add(nk2Sizer, 0, wx.ALL|wx.EXPAND, 2)

		column2Sizer.Add(lspSizer, 0, wx.ALL|wx.EXPAND, 2)
		column2Sizer.Add(lsrSizer, 0, wx.ALL|wx.EXPAND, 2)
		column2Sizer.Add(lsySizer, 0, wx.ALL|wx.EXPAND, 2)
		column2Sizer.Add(lebSizer, 0, wx.ALL|wx.EXPAND, 2)
		column2Sizer.Add(lwySizer, 0, wx.ALL|wx.EXPAND, 2)
		column2Sizer.Add(lwrSizer, 0, wx.ALL|wx.EXPAND, 2)

		column3Sizer.Add(rspSizer, 0, wx.ALL|wx.EXPAND, 2)
		column3Sizer.Add(rsrSizer, 0, wx.ALL|wx.EXPAND, 2)
		column3Sizer.Add(rsySizer, 0, wx.ALL|wx.EXPAND, 2)
		column3Sizer.Add(rebSizer, 0, wx.ALL|wx.EXPAND, 2)
		column3Sizer.Add(rwySizer, 0, wx.ALL|wx.EXPAND, 2)
		column3Sizer.Add(rwrSizer, 0, wx.ALL|wx.EXPAND, 2)
	
		column4Sizer.Add(lhySizer, 0, wx.ALL|wx.EXPAND, 2)
		column4Sizer.Add(lhrSizer, 0, wx.ALL|wx.EXPAND, 2)
		column4Sizer.Add(lhpSizer, 0, wx.ALL|wx.EXPAND, 2)
		column4Sizer.Add(lknSizer, 0, wx.ALL|wx.EXPAND, 2)
		column4Sizer.Add(lapSizer, 0, wx.ALL|wx.EXPAND, 2)
		column4Sizer.Add(larSizer, 0, wx.ALL|wx.EXPAND, 2)

		column5Sizer.Add(rhySizer, 0, wx.ALL|wx.EXPAND, 2)
		column5Sizer.Add(rhrSizer, 0, wx.ALL|wx.EXPAND, 2)
		column5Sizer.Add(rhpSizer, 0, wx.ALL|wx.EXPAND, 2)
		column5Sizer.Add(rknSizer, 0, wx.ALL|wx.EXPAND, 2)
		column5Sizer.Add(rapSizer, 0, wx.ALL|wx.EXPAND, 2)
		column5Sizer.Add(rarSizer, 0, wx.ALL|wx.EXPAND, 2)

		column6Sizer.Add(rf1Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column6Sizer.Add(rf2Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column6Sizer.Add(rf3Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column6Sizer.Add(rf4Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column6Sizer.Add(rf5Sizer, 0, wx.ALL|wx.EXPAND, 2)

		column7Sizer.Add(lf1Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column7Sizer.Add(lf2Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column7Sizer.Add(lf3Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column7Sizer.Add(lf4Sizer, 0, wx.ALL|wx.EXPAND, 2)
		column7Sizer.Add(lf5Sizer, 0, wx.ALL|wx.EXPAND, 2)

		# Add the column sizers to the zeroed sizer
		zeroedSizer.Add(column1Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column2Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column3Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column4Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column5Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column6Sizer, 0, wx.ALL|wx.EXPAND, 3)
		zeroedSizer.Add(column7Sizer, 0, wx.ALL|wx.EXPAND, 3)
		
		# Add all sections to main sizer
		mainSizer.Add(wx.StaticText(panel, wx.ID_ANY, 'Zeroed Status'), 0, wx.CENTER, 3)
		mainSizer.Add(wx.StaticLine(panel,), 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(zeroedSizer, 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(wx.StaticLine(panel,), 0, wx.ALL|wx.EXPAND, 3)
		mainSizer.Add(wx.StaticText(panel, wx.ID_ANY, 'Force/Torque Sensors'), 0, wx.CENTER, 3)
		panel.SetSizer(mainSizer)
		mainSizer.Fit(self)

		# Set events
		self.Bind(wx.EVT_MENU, self.OnAbout, menuAbout) # bind menuAbout object to select menu item event, and call the function "OnAbout"
		self.Bind(wx.EVT_MENU, self.OnExit, menuExit) # bind menuExit object to select menu item event, and call the function "OnExit"

	# A message dialog box with an OK button
	def OnAbout(self, event):
		dlg = wx.MessageDialog(self, "Feedback\n1.0\n\nA program to display feedback for all of Hubo's sensors and motors.", "About Feedback", wx.OK) # create MessageDialog widget with (Text to be displayed, frame title, Id) parameters. wx.OK is standard ID in wxWidgets
		dlg.ShowModal() # display dialog box
		dlg.Destroy() # destory it when finished

	# Close the program with "Exit" is selected from the File menu
	def OnExit(self, event):
		self.Close(True) # close the frame

if __name__ == '__main__':
	app = wx.App(False) # create a new app. False means don't redirect stdout/stderr to new window
	frame = Feedback() # frame constructor. Frame(parent, Id, title)
	frame.Show() # show frame
	app.MainLoop() # start app's main loop, handling events
