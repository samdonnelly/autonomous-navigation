#================================================================================
# File: main.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: TerraPlanner entry 
#
# Date: 2025-04-04 
#================================================================================


#================================================================================
# Includes 

import sys
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QFrame, QPushButton, QFileDialog, QMessageBox
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import Qt

#================================================================================


#================================================================================
# Main 

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Vehicle Data Visualizer")
        self.setMinimumSize(1000, 600)
        self.coordinates = \
        [
            [51.14256236, -114.04202439]   # Default 
        ]

        # Create central widget and main layout
        central_widget = QWidget()
        outer_layout = QVBoxLayout()

        # Header panel with file upload button
        header_layout = QHBoxLayout()
        upload_button = QPushButton("Upload Path File")
        upload_button.clicked.connect(self.load_path_file)
        header_layout.addWidget(upload_button)
        outer_layout.addLayout(header_layout)

        main_layout = QHBoxLayout()

        # Left section (map view)
        left_section = QFrame()
        left_layout = QVBoxLayout()
        self.map_view = QWebEngineView()
        self.update_map()
        left_layout.addWidget(self.map_view)
        left_section.setLayout(left_layout)
        left_section.setMinimumWidth(600)

        # Right layout with two stacked sections
        right_layout = QVBoxLayout()

        top_right_section = QFrame()
        top_right_section.setFrameShape(QFrame.Shape.StyledPanel)
        top_right_section.setMinimumHeight(250)

        bottom_right_section = QFrame()
        bottom_right_section.setFrameShape(QFrame.Shape.StyledPanel)
        bottom_right_section.setMinimumHeight(250)

        right_layout.addWidget(top_right_section)
        right_layout.addWidget(bottom_right_section)

        # Add to main layout
        main_layout.addWidget(left_section, 2)
        main_layout.addLayout(right_layout, 1)

        outer_layout.addLayout(main_layout)
        central_widget.setLayout(outer_layout)
        self.setCentralWidget(central_widget)

    def load_path_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Path File", "", "Text Files (*.txt)")
        if file_path:
            new_coords = []
            invalid = False
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        lat_str, lon_str = line.split(',')
                        lat = float(lat_str.strip())
                        lon = float(lon_str.strip())
                        new_coords.append([lat, lon])
                    except ValueError:
                        invalid = True
            if new_coords:
                self.coordinates = new_coords
                self.update_map()
            if invalid:
                QMessageBox.warning(
                    self,
                    "Invalid File Format",
                    "One or more pieces of data in the path file are invalid. Please check formatting."
                )

    def update_map(self):
        html = self.generate_map_html()
        self.map_view.setHtml(html)

    def generate_map_html(self):
        coords_js = json.dumps(self.coordinates)
        return f"""
        <!DOCTYPE html>
        <html>
            <head>
                <meta charset='utf-8'>
                <title>Map</title>
                <meta name='viewport' content='width=device-width, initial-scale=1.0'>
                <link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css'/>
                <style>
                    html, body, #map {{ height: 100%; margin: 0; }}
                </style>
            </head>
            <body>
                <div id='map'></div>
                <script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>
                <script>
                    var waypoints = {coords_js};
                    var map = L.map('map').setView(waypoints[0], 17);
                    L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                        maxZoom: 19,
                    }}).addTo(map);
                    for (let i = 0; i < waypoints.length; i++) {{
                        L.marker(waypoints[i]).addTo(map);
                    }}
                    L.polyline(waypoints, {{color: 'yellow'}}).addTo(map);
                </script>
            </body>
        </html>
        """


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

#================================================================================
