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

#================================================================================


#================================================================================
# Main window 

class MainWindow(QMainWindow): 
    def __init__(self): 
        super().__init__() 

        self.setWindowTitle("Vehicle Data Visualization") 

        button = QPushButton("Push!") 
        button.setCheckable(True) 
        button.clicked.connect(self.the_button_was_clicked) 
        button.clicked.connect(self.the_button_was_toggled) 

        self.setFixedSize(QSize(400, 300)) 

        self.setCentralWidget(button) 

    def the_button_was_clicked(self): 
        print("Clicked!") 

    def the_button_was_toggled(self, checked): 
        print("Checked?", checked) 

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
