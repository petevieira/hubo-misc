#!/usr/bin/python
import wx

def makeBold(staticTextWidget):
	font = staticTextWidget.GetFont()
	font.SetWeight(wx.FONTWEIGHT_BOLD)
	staticTextWidget.SetFont(font)

