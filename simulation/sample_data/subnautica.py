#================================================================================
# File: subnautica.py 
# 
# Author: Sam Donnelly (samueldonnelly11@gmail.com)
# 
# Description: This file contains data based off the video game "Subnautica". 
#              This data is used with the "trilateration.py" script to generate 
#              a map showing the relative location of points of interest within 
#              the game. To effectively use this for the game, the user must 
#              create at least three reference points (i.e. with a known 
#              location) and manually populate the points of interest list 
#              with distances to each reference point from the users current 
#              location within the game. For example, if you want to mark the 
#              location of a wreck, place yourself at the wreck site within the 
#              game, then record the distance seen to each reference point and 
#              put those distances in the list. The number recorded distances 
#              must match the number of defined reference points. 
#
# Date: December 4, 2024 
#================================================================================


#================================================================================
# Imports 

import sys 
sys.path.append("../simulation") 
from trilateration import trilateration 

#================================================================================


#================================================================================
# Data 

# The following reference coordinates were calculated manually relative to one 
# another using the life pod (reference #1) as the map origin. 

# Reference coordinate 1 - Life Pod 
x1 = 0.0 
y1 = 0.0 

# Reference coordinate 2 - Gun Island Beach 
x2 = 292.1843 
y2 = 1165.9470 

# Reference coordinate 3 - Auora Entrance 
x3 = 1077.8830 
y3 = 214.4043 

# Known coordinates 
Pi = \
(
    (x1, y1, "Life Pod"),        # Coordinate 1 - Life Pod 
    (x2, y2, "Gun Island"),      # Coordinate 2 - Gun Island Beach 
    (x3, y3, "Auora Entrance")   # Coordinate 3 - Auora Entrance 
)

# Initial guesses for finding the location of a point of interest 
Pg1 = ((x1 + x2 + x3) / 2, (y1 + y2 + y3) / 2) 
Pg2 = (-Pg1[0], -Pg1[1]) 

# Location (distances) of points of interest 
di = \
(
    # Wrecks 
    (449, 1489, 1535, Pg1, "W0"), 
    (1275, 2346, 2258, Pg1, "W1"), 
    (745, 1918, 1616, Pg1, "W2"), 
    (1539, 545, 1205, Pg1, "W3"), 
    (1198, 777, 568, Pg1, "W4"), 
    # Caves 
    # Materials 
    # Islands 
    # (1203, 0, 1234, "I0"),         # Gun Island - already set as a landmark 
    (1177, 2334, 2122, Pg1, "I1"),   # Habitat Island 
    # Habitats 
    # Discoveries 
    (1388, 2274, 2384, Pg2, "D0"),   # Alien Research Lab (Orange tablet) & Fossil 
    (1672, 1048, 922, Pg1, "D1"),     # Entrance to underground network (depth ~400m) 
)

#================================================================================


#================================================================================
# Test the data 

trilateration(Pi, di) 

#================================================================================
