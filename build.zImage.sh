#!/bin/sh
 
CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+2}')

START_TIME=`date +%s`
echo make -j$CPU_JOB_NUM
echo
make -j4 ARCH=arm zImage
if [ $? != 0 ] ; then
	exit $?
fi

END_TIME=`date +%s`

echo "Total compile time is $((($END_TIME-$START_TIME)/60)) minutes $((($END_TIME-$START_TIME)%60)) seconds"

cp -a arch/arm/boot/zImage /var/www
