#! /bin/bash

# HOSTBOARD FIRMWARE UPDATING SCRIPT
# uses ssh if available
# otherwise uses ckermit



REMOTE_SSH=$( ifconfig | grep 192.168.7. )

SSH_TARGET="hostboard"


SOURCE_FOLDER=Release




# local stuff

LAST_FIRMWARE_HASH=$( sha256sum WildhornAV_CM4.elf | awk '{print $1}' )

echo "last firmware hash [local]: "$LAST_FIRMWARE_HASH

# use ssh
echo "using SSH"

ssh $SSH_TARGET " kill -9 \$( pidof nordend_monitor )"

cp  ../../CA7/nordend_monitor/nordend_monitor nordend_monitor
bzip2 -f nordend_monitor
echo "monitor deflated"
scp nordend_monitor.bz2 $SSH_TARGET:~/
echo "monitor sent"
ssh $SSH_TARGET "bunzip2 -f nordend_monitor.bz2"
echo "monitor inflated"
scp rc.local $SSH_TARGET:/etc/rc.local
ssh $SSH_TARGET "chmod +x /etc/rc.local"
echo "tools sent"

cp  ../$SOURCE_FOLDER/WildhornAV_CM4.elf WildhornAV_CM4.elf
bzip2 -f WildhornAV_CM4.elf
echo "firmware deflated"

scp WildhornAV_CM4.elf.bz2 $SSH_TARGET:~/
echo "firmware sent"

ssh $SSH_TARGET "bunzip2 -f  WildhornAV_CM4.elf.bz2"
echo "firmware inflated"

ssh $SSH_TARGET "mkdir /lib/firmware/"

ssh $SSH_TARGET "cp WildhornAV_CM4.elf /lib/firmware/rproc-m4-fw"
echo "firmware installed"	

REMOTE_HASH_CHECK=$( ssh $SSH_TARGET "sha256sum /lib/firmware/rproc-m4-fw" | awk '{print $1}' )
echo "remote installed firmware hash: "$REMOTE_HASH_CHECK



LOCAL_HASH_CHECK=$( sha256sum ../$SOURCE_FOLDER/WildhornAV_CM4.elf | awk '{print $1}' )
echo "local latest firmware hash: "$LOCAL_HASH_CHECK

if [[ $REMOTE_HASH_CHECK == $LOCAL_HASH_CHECK ]]; then
	echo "match, success!"
else
	echo "error, install unsucessul"
	exit 1 
fi

echo "terminating, rebooting hostboard"

ssh $SSH_TARGET "shutdown -r 0"



