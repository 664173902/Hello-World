/**
 * @file    jni_util.c
 *          
 * @author  PanJianPing <paddy.pan@newpostech.com>.
 *          
 * @date    2016-03-08
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <android/log.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

#include "jni_util.h"

/**
 * @fn          jstring_to_char
 * @brief       Java字符串转c字符串
 * @param in
 * @param out   result
 * @return
 * @author      PanJianPing
 * @date        2016-03-19
 * @note
 */
void jstring_to_char(JNIEnv *env, jstring jstr, const uint32_t len, char *result) {
    (*env)->GetStringUTFRegion(env, jstr, 0, len - 1, result);
    result[len] = 0x00;
}


/**
 * @fn          hexdump
 * @brief       打印字节数组和内存内容
 * @param in    
 * @param out   
 * @return      
 * @author      PanJianPing
 * @date        2016-03-09
 * @note        
 */
#ifdef DEBUG
void dbg_print_memory(const char * pszTitle,const unsigned char *pData, const int iData) {
    unsigned int i;
	char szHexTemp[266];
    char szBinTemp[60];
    LOGD("%s(size = %d)\r\n", pszTitle, iData);
    szHexTemp[0] = 0x00;
    szBinTemp[0] = 0x00;
    
    for (i = 0; i < iData; i++) {
        sprintf(szHexTemp + strlen(szHexTemp), "%02X ", pData[i]);
        szBinTemp[strlen(szBinTemp)+1] = 0x00;

        if (isprint(pData[i]))
			szBinTemp[strlen(szBinTemp)] = pData[i];
		else
			szBinTemp[strlen(szBinTemp)] = '.';
        
        
        if (((i + 1) % 16 == 0) || (i + 1 == iData)) {
            while (strlen(szHexTemp) < 48) {
                strcat(szHexTemp, " ");
            }
            LOGD("     %s    %s", szHexTemp, szBinTemp);
            szHexTemp[0] = 0x00;
			szBinTemp[0] = 0x00;	
        }
    }
}
#endif


