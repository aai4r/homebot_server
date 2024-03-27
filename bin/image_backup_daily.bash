#!/bin/bash

HOST=192.168.0.9
USER=admin
PASSWD=wjdwns12

echo "connect ftp server.."

ftp -nv <<!EOF
open $HOST
user $USER $PASSWD
cd DVR-video
mkdir hancom_qi_1
cd hancom_qi_1
lcd /data1/video_backup
prompt off
mput *.avi
bye
!EOF

#echo "delete local image files.."
#rm -rf /data1/video_backup/*
#echo "done."

cd /data1/video_backup
mkdir $DATE_NOW
mv *.avi $DATE_NOW

exit 0