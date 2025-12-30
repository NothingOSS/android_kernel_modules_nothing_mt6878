###############################################################################
# Generally Android.mk can not get KConfig setting
# we can use this way to get
# include the final KConfig
# but there is no $(AUTO_CONF) at the first time (no out folder) when make
#
#ifneq (,$(wildcard $(AUTO_CONF)))
#include $(AUTO_CONF)
#include $(CLEAR_VARS)
#endif

###############################################################################
###############################################################################
# Generally Android.mk can not get KConfig setting                            #
#                                                                             #
# do not have any KConfig checking in Android.mk                              #
# do not have any KConfig checking in Android.mk                              #
# do not have any KConfig checking in Android.mk                              #
#                                                                             #
# e.g. ifeq ($(CONFIG_MTK_COMBO_WIFI), m)                                     #
#          xxxx                                                               #
#      endif                                                                  #
#                                                                             #
# e.g. ifneq ($(filter "MT6632",$(CONFIG_MTK_COMBO_CHIP)),)                   #
#          xxxx                                                               #
#      endif                                                                  #
#                                                                             #
# All the KConfig checking should move to Makefile for each module            #
# All the KConfig checking should move to Makefile for each module            #
# All the KConfig checking should move to Makefile for each module            #
#                                                                             #
###############################################################################
###############################################################################

LOCAL_PATH := $(call my-dir)

BT_PLATFORM:=$(subst MTK_CONSYS_MT,,$(MTK_BT_CHIP))
$(info [BT_Drv] MTK_BT_SUPPORT = $(MTK_BT_SUPPORT))
$(info [BT_Drv] MTK_BT_CHIP = $(MTK_BT_CHIP))

ifeq ($(MTK_BT_SUPPORT),yes)
#ifneq ($(filter MTK_MT79%, $(MTK_BT_CHIP)),)

include $(CLEAR_VARS)
LOCAL_MODULE := btmtk_uart_unify_mt6653.ko
LOCAL_REQUIRED_MODULES := conninfra_mt6653.ko
LOCAL_REQUIRED_MODULES += connfem.ko
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := mtk
LOCAL_INIT_RC := init.btmtk_uart.rc
LOCAL_SRC_FILES := $(patsubst $(LOCAL_PATH)/%,%,$(shell find $(LOCAL_PATH) -type f -name '*.[cho]')) Makefile

include $(MTK_KERNEL_MODULE)
include $(LOCAL_PATH)/uart_daemon/Android.mk
#endif
endif
