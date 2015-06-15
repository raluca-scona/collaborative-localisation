#!/bin/bash

cd $HOME
START_TIME=$SECONDS
FL1=0
FL2=0
FL3=0
FL4=0

xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR 2 3 0 0 0 0" &

while [ $(($SECONDS - $START_TIME)) -lt 600 ]; do

	if [ $(($SECONDS - $START_TIME)) -gt 100 ] && [ "$FL1" == "0" ]; then
		echo "Changing direction 1"
		FL1=1
        	xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR 3 2 0 0 0 0" &
  
	elif [ $(($SECONDS - $START_TIME)) -gt 250 ] && [ "$FL2" == "0" ]; then
		echo "Changing direction 2"
		FL2=1
        	xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR -2 4 0 0 0.1 0" &

	elif [ $(($SECONDS - $START_TIME)) -gt 350 ] && [ "$FL3" == "0" ]; then
		echo "Changing direction 3"
		FL3=1
        	xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR 5 3 0 0.1 -0.1 0.2" &

	elif [ $(($SECONDS - $START_TIME)) -gt 500 ] && [ "$FL4" == "0" ]; then
		echo "Changing direction 4"
		FL4=1
        	xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR 3 3 0 0 0 0" &
	fi
		
done
xterm -hold -e "rosrun uwsim setVehicleVelocity \dataNavigator_AZOR 0 0 0 0 0 0" &
xterm -hold -e "rosrun uwsim setVehiclePosition \dataNavigator_AZOR 2 2 3 0 0 0" &

exit 0
