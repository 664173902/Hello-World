LOCAL_PATH := $(call my-dir)
SDK_PATH := /home/panjp/workspace/toolchain/sdk_pos/

include $(CLEAR_VARS)
LOCAL_MODULE := ped-prebuilt
LOCAL_SRC_FILES := $(SDK_PATH)/lib/libped.so
LOCAL_EXPORT_C_INCLUDES := $(SDK_PATH)/include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := pos-prebuilt
LOCAL_SRC_FILES := $(SDK_PATH)/lib/libpos.so
LOCAL_EXPORT_C_INCLUDES := $(SDK_PATH)/include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjni_serial
LOCAL_SRC_FILES := jni_util.c \
                   uart.c \
                   appuart.c \
                   com_newpos_uart_SerialPort.c \
#LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_C_INCLUDES := $(SDK_PATH)/include

LOCAL_SHARED_LIBRARIES := ped-prebuilt pos-prebuilt
LOCAL_LDLIBS += -L$(SYSROOT)/usr/lib -llog
LOCAL_PROGUARD_ENABLED := disabled 
include $(BUILD_SHARED_LIBRARY)