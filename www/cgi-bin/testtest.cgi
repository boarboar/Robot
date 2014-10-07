#!/bin/ash 
echo "Test start..."
	
	(
		read -r -t 4 < /dev/ttyATH0
#		echo $REPLY > /tmp/drvcon.tmp
		echo "R1="$REPLY
		let _R=$REPLY
		echo "R2="$_R
	) &
	
	echo "Test" > /dev/ttyATH0
	wait     
	echo "R3="$_R
	
#	Test start...
#R1="R":"2",10049;0,0;0,0;0
#R2="R":"2",10049;0,0;0,0;0
#R3=
