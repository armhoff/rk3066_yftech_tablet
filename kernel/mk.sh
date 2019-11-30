#!/bin/bash
if [ "$1" == "" ]
then
  CFG_B=mr13
else
  CFG_B=$1
fi
INPUT=arch/arm/boot/Image
CFG_E=config
CFG_T=arch/arm/mach-rk30/config/$CFG_B.txt
CFG_C=arch/arm/mach-rk30/config/config.c
CFG_K=arch/arm/configs/rk3066_yf_defconfig

OUTPUT=$CFG_B.img

if [ ! -f $CFG_T ]; then
  echo no config file found for $CFG_B
  exit
fi
if [ $CFG_C -nt $CFG_E ]
then
  echo generating $CFG_E
  gcc $CFG_C -o $CFG_E
fi

if [ $CFG_K -nt .config ]
then
  echo generating kernel config
  cp $CFG_K .config
fi

echo generating kernel
make Image
if [ $? -eq 0 ]
then
  if [[ $INPUT -nt $OUTPUT || $CFG_T -nt $OUTPUT || $CFG_E -nt $OUTPUT ]]
  then
    echo generating image
    ./$CFG_E $CFG_T $INPUT $CFG_B.kmg
    if [ ! $? -eq 0 ]; then
      echo failed to apply config
      exit
    fi
    ./mkkrnlimg $CFG_B.kmg $OUTPUT
    rm $CFG_B.kmg
  fi
  rm  kernel.img >/dev/null 2>&1
  ln -s $OUTPUT kernel.img
fi
