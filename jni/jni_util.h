/**
 * @file    jni_util.h
 *          
 * @author  PanJianPing <paddy.pan@newpostech.com>.
 *          
 * @date    2016-03-17
 *          
 * @brief   
 *          
 * @note    
 *          
 * Modification history
 * ----------------------------------------------------------------------------
 * Date         Version  Author       History
 * ----------------------------------------------------------------------------
 *
 */

#include <jni.h>
#include <android/log.h>
#include <stdint.h>

#ifndef __UTIL_H__
#define __UTIL_H__

#ifdef __cplusplus 
extern "C" { 
#endif

#define DEBUG
#define LOG_TAG    "__NPUMSTEST__"

#ifdef  DEBUG
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#else
#define LOGD(...)   if(0){}
#endif

#define LOGE(...)   __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define LOGI(...)   __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

#define GET_ARRAY_LEN(array,len){len = (sizeof(array) / sizeof(array[0]));}

void jstring_to_char(JNIEnv *menv, jstring jstr, const uint32_t len, char *result);

#ifdef DEBUG
void dbg_print_memory(const char * pszTitle,const unsigned char *pData, const int iData);
#else
#define dbg_print_memory(pszTitle, pdata, iData)
#endif

#ifdef __cplusplus 
}
#endif 

#endif

