# top level project rules for the u8800 project
#
LOCAL_DIR := $(GET_LOCAL_DIR)

TARGET := u8800

MODULES += app/aboot

#INFO debug level
DEBUG := INFO

#DEFINES += WITH_DEBUG_DCC=1
#DEFINES += WITH_DEBUG_UART=1
DEFINES += WITH_DEBUG_FBCON=1
