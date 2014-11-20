#!/bin/ash 
#GET implementation
echo "HTTP/1.0 200 OK"
echo -e "Content-Type:  application/json\n"

for _VAR in ${QUERY_STRING}
do
 _VARN=$(echo ${_VAR} | cut -d= -f1) 
 if [ "${_VARN}" = "SERIAL_DATA" ]
 then
	_VARV=$(echo ${_VAR} | cut -d= -f2)
	_DEC=$(printf "%b" "${_VARV//\%/\x}" | tr -d '\r\n')
#	echo -n '{"C":"'$_DEC'","R":"'
	echo -n '{"C":"'$_DEC'",'	
	(
		read -r -t 1 < /dev/ttyATH0
		echo $REPLY > /tmp/drvcon.tmp
	) &
	read _T1 trash < /proc/uptime
	echo $_DEC > /dev/ttyATH0
	wait     
	read _T2 trash < /proc/uptime
	cat /tmp/drvcon.tmp | tr -d '\r\n'
#	echo '"}'
	echo '"X":"0"}'
	echo -n "$_T1 $_T2 $_DEC : " >> /var/log/drive.log
	cat /tmp/drvcon.tmp >> /var/log/drive.log	
 fi 
done

unset _VAR
unset _VARV
unset _VARN
unset _DEC


#_T=$(cat /proc/uptime | cut -d' ' -f1 | tr -d '.')
#_PL=$(( ($_T)%255 ))
#_PR=$(( (($_T)*3)%255 ))
#_RL=$(( ($_T)%600 ))
#_RR=$(( (($_T)*3)%600 ))
#echo '{"C":"CMD","PL":"'$_PL'","PR":"'$_PR'","RL":"'$_RL'","RR":"'$_RR'"}' 


 