rostopic pub -1 /System api_application/System "{header: auto, command: 1}"

. ../Controller/SetQuadcopters.sh
. ../Controller/SetFormation1.sh
. ../Controller/BuildFormation.sh
