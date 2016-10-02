# collaborative-localisation

### Building:
 - ``cd ~/catkin_ws/src/``
 - ``git clone git@github.com:raluca-scona/collaborative-localisation.git``
 - Build by running ``catkin_make`` from ~/catkin_ws

### Loading the scene:
``rosrun uwsim uwsim --configfile scenes/threerobots.xml``

The surface vehicle is named "TOP" and one of the bottom vehicles is named "AZOR".

To get AZOR to move forward:

``rosrun uwsim setVehicleVelocity /dataNavigator_AZOR 0.1 0 0 0 0 0``

### Running collaborative localisation:
   All localisation algorithms use dead reckoning as motion model and the USBL measurements as correction

 - Simulate an USBL reading from the TOP vehicle to AZOR:
 
	``rosrun multi_robot_localisation getUSBLReadingForAzor.py``
 - To run one of the localisation algorithms:
   
  Particle Filter:
    - using both range and bearing USBL estimates:
	``rosrun multi_robot_localisation PFAzor.py``
    - using only range USBL estimates:
	``rosrun multi_robot_localisation PFRangeAzor.py``

  Non-linear Least Squares:
    - using both range and bearing USBL estimates:
	``rosrun multi_robot_localisation NLSAzor.py``
    - using only range USBL estimates:
	``rosrun multi_robot_localisation NLSRangeAzor.py``

  Interval Calculus:
    - using both range and bearing USBL estimates:
	``rosrun multi_robot_localisation ICAzor.py``
    - using only range USBL estimates:
	``rosrun multi_robot_localisation ICRangeAzor.py``
