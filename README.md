# Autonomous Navigation 

Unmanned boat used to autonomously navigate short range waypoint missions on open bodies of water. This project is comprised of two main systems: boat and ground station. The boat performs the missions while the ground station is used to command the boat from shore. Both run on the STM32F4 and utilize the <a href="https://github.com/samdonnelly/STM32F4-driver-library">STM32F4-driver-library</a> to talk with external devices. 

Boat Code: <a href="https://github.com/samdonnelly/autonomous-navigation/tree/main/auto_nav/sources/boat">src</a>, <a href="https://github.com/samdonnelly/autonomous-navigation/tree/main/auto_nav/headers/boat">inc</a> \
Ground Station Code: <a href="https://github.com/samdonnelly/autonomous-navigation/tree/main/auto_nav/sources/ground_station">src</a>, <a href="https://github.com/samdonnelly/autonomous-navigation/tree/main/auto_nav/headers/ground_station">inc</a> \
Project Build and Background: <a href="https://samueldonnelly11.wixsite.com/builds/autonomous-boat">Link</a> 

## Simulation 

Scripts used to test navigation calculations before implementing them in the real system. 

Simulation Scripts: <a href="https://github.com/samdonnelly/autonomous-navigation/tree/main/simulation">folder</a> 
