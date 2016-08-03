#!/bin/sh

OSVER=2.6.27

if [ `id | cut -d= -f2 | cut -d\( -f1` -ne 0 ]; then 
   echo "$0 need root privileges"
   exit 1
fi

if [ ! -f "${1}" -o -z "${2}" ]; then
	echo "input virtual disk image and domainU's ID"
	exit 1;
elif [ ! "`xm list | awk '{printf("%s\n", $2)}' | grep ${2}`" ]; then
	echo "not exist domainU"
	exit 1;
fi;

MOUNT_DIR="/tmp/vm${2}"
PROC_DIR="/tmp/vm${2}/proc"

mkdir ${MOUNT_DIR} 

./mount_vdi "${1}" "${MOUNT_DIR}"
../sprocfs/${OSVER}/shadow_procfs "${PROC_DIR}" "${2}"
if [ $? -ne 0 ]; then
    ./umount_vdi "${MOUNT_DIR}"
    if [ $? -ne 0 ]; then
        exit
    fi
    rm -rf ${MOUNT_DIR}
    exit
fi

insmod ../kmod/shadowfs.ko root_dir="${MOUNT_DIR}"

(cd /;
bash)

rmmod shadowfs

umount "${PROC_DIR}" 
if [ $? -ne 0 ]; then
    exit
fi

./umount_vdi "${MOUNT_DIR}"
if [ $? -ne 0 ]; then
    exit
fi

rm -rf ${MOUNT_DIR}