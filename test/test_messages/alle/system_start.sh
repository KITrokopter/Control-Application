rostopic pub -1 /System api_application/System "{header: auto, command: 1}"

rosrun control_application SetQuadcopters.sh
rosrun control_application SetFormation1.sh
rosrun control_application BuildFormation.sh
#. ../Controller/SetQuadcopters.sh
#. ../Controller/SetFormation1.sh
#. ../Controller/BuildFormation.sh
