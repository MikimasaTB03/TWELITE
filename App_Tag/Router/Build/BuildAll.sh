#!/bin/sh

make TWE_CHIP_MODEL=JN5164 clean
make TWE_CHIP_MODEL=JN5164 all -j 4
