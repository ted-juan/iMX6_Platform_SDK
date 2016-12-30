include $(SDK_ROOT)/mk/common.mk


# Export the target library name to all child make invocations.
TARGET_LIB_NAME = sdk
export TARGET_LIB_NAME

ifeq "$(FREERTOS)" "1"
SUBDIRS = \
	drivers
else
SUBDIRS = \
	drivers \
	common \
	core \
	utility
endif

include $(SDK_ROOT)/mk/targets.mk
