#!/bin/bash

BOARD_MCU=$1
UPLOAD_PORT=$2
UPLOAD_SPEED=$3
SOURCE=$4

pymcuprog erase -t uart -d $BOARD_MCU -u $UPLOAD_PORT
pymcuprog write -t uart -d $BOARD_MCU -u $UPLOAD_PORT -c $UPLOAD_SPEED -f $SOURCE
