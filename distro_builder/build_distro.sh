#!/bin/bash

#cd ../CA7/DeviceTree/
#patch -p3 --forward --no-backup-if-mismatch < 0001-add-uart-bias.patch
#python3 dts_patcher.py
#cd -

cd ../distribution-package

MACHINE=stm32mp1-wildhornav DISTRO=openstlinux-ert EULA_stm32mp1wildhornav=1 source layers/meta-st/scripts/envsetup.sh --no-ui

bitbake ert-image-core

echo "done!"
