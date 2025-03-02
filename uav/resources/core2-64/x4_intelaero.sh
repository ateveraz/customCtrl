#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./customCtrl_rt
else
	EXEC=./customCtrl_nrt
fi

$EXEC -n Drone_0 -a 172.26.213.62 -p 9000 -l /tmp -x setup_x4_intelaero.xml -t aero
