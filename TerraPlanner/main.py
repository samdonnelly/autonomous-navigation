#================================================================================
# File: main.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: GUI entry point 
# 
# Notes: 
# - Reference: https://www.pythonguis.com/pyqt6-tutorial/ 
#   - Place: PyQt6 Signals, Slots & Events - Events 
#
# Date: 2025-04-04 
#================================================================================

#================================================================================
# Includes 

# Library 
from PyQt6.QtCore import QSize, Qt 
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QVBoxLayout, QWidget 

import sys   # For command line arguments 
from random import choice 

#================================================================================


#================================================================================
# Data 
#================================================================================


#================================================================================
# Main window 

class MainWindow(QMainWindow): 
    def __init__(self): 
        super().__init__() 

        self.setWindowTitle("Vehicle Data Visualization") 

        self.label = QLabel() 

        self.input = QLineEdit() 
        self.input.textChanged.connect(self.label.setText) 

        layout = QVBoxLayout() 
        layout.addWidget(self.input) 
        layout.addWidget(self.label) 

        container = QWidget() 
        container.setLayout(layout) 

        # Set the central widget of the window 
        self.setCentralWidget(container) 

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
