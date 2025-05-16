#================================================================================
# File: main.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: GUI entry point 
# 
# Notes: 
# - Reference: https://www.pythonguis.com/pyqt6-tutorial/ 
#   - Place: PyQt6 Signals, Slots & Events - Mouse events 
#
# Date: 2025-04-04 
#================================================================================

#================================================================================
# Includes 

# Library 
from PyQt6.QtCore import Qt 
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QTextEdit 

import sys   # For command line arguments 

#================================================================================


#================================================================================
# Data 
#================================================================================


#================================================================================
# Main window 

class MainWindow(QMainWindow): 
    def __init__(self): 
        super().__init__() 
        self.setMouseTracking(True) 
        self.label = QLabel("Click in this Window") 
        self.label.setMouseTracking(True) 
        self.setCentralWidget(self.label) 

    def mouseMoveEvent(self, e): 
        self.label.setText("mouseMoveEvent") 
        
    def mousePressEvent(self, e): 
        self.label.setText("mousePressEvent") 
        
    def mouseReleaseEvent(self, e): 
        self.label.setText("mouseReleaseEvent") 
        
    def mouseDoubleClickEvent(self, e): 
        self.label.setText("mouseDoubleClickEvent") 

#================================================================================


#================================================================================
# Main 

# Only one QApplication instance is need per application. You can pass sys.argv 
# to allow command line arguments for the app. If these aren't needed then you 
# pass "[]" instead. 
app = QApplication(sys.argv) 

# Create a Qt widget (window) 
window = MainWindow() 
window.show() 

# Start the event loop 
app.exec() 

#================================================================================
