#================================================================================
# File: main.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: GUI entry point 
#
# Date: 2025-04-04 
#================================================================================

#================================================================================
# Includes 

# Library 
from PyQt6.QtCore import QSize, Qt 
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton 

import sys   # For command line arguments 
from random import choice 

#================================================================================


#================================================================================
# Data 

window_titles = \
[
    'My App', 
    'My App', 
    'Still My App', 
    'Still My App', 
    'What on earth', 
    'What on earth', 
    'This is surprising', 
    'This is surprising', 
    'Something went wrong' 
]

#================================================================================


#================================================================================
# Main window 

class MainWindow(QMainWindow): 
    def __init__(self): 
        super().__init__() 

        self.n_times_clicked = 0 

        self.setWindowTitle("Vehicle Data Visualization") 

        self.button = QPushButton("Push!") 
        self.button.clicked.connect(self.the_button_was_clicked) 

        self.windowTitleChanged.connect(self.the_window_title_changed) 

        self.setFixedSize(QSize(400, 300)) 

        self.setCentralWidget(self.button) 

    def the_button_was_clicked(self): 
        print("Clicked!") 
        new_window_title = choice(window_titles) 
        print("Setting title: %s" % new_window_title) 
        self.setWindowTitle(new_window_title) 

    def the_window_title_changed(self, window_title): 
        print("Window title changed: %s" % window_title) 

        if window_title == 'Something went wrong': 
            self.button.setDisabled(True) 

#================================================================================


#================================================================================
# Main 

# Only one QApplication instance is need per application. You can pass sys.argv 
# to allow command line arguments for the app. If these aren't needed then you 
# pass "[]" instead. 
app = QApplication(sys.argv) 

# Create a Qt widget (window) 
# window = QWidget() 
window = MainWindow() 
window.show() 

# Start the event loop 
app.exec() 

#================================================================================
