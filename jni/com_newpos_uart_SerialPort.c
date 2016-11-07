#include <stdlib.h>
#include <errno.h>

#include "com_newpos_uart_SerialPort.h"
#include "jni_util.h"
#include "serial.h"
#include "ped.h"
#include "vendor.h"

static int glbPortNum = -1;

extern void beep(unsigned int frequency, unsigned int keepms);

/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    open
 * Signature: (ILjava/lang/String;)Ljava/io/FileDescriptor;
 */
JNIEXPORT jobject JNICALL Java_com_newpos_uart_SerialPort_open
(JNIEnv *env, jobject thiz, jint portNum, jstring portParm)
{
    int retval = -1;
    int portFD = -1;
    jobject mFileDescriptor;

    int len = (*env)->GetStringUTFLength(env, portParm);
    unsigned char param[len + 1];
    jstring_to_char(env, portParm, len + 1, param);
    LOGD("portNum:%d; portParam:%s", portNum, param);

    retval = portOpen(portNum, param);
    if (retval < 0) {
        LOGE("portOpen err");
        return NULL;
    }

    portFD = GetPortFd(portNum);

    LOGD("portFD =%d",portFD);

    /* Create a corresponding file descriptor */
    jclass cFileDescriptor = (*env)->FindClass(env, "java/io/FileDescriptor");
    jmethodID iFileDescriptor = (*env)->GetMethodID(env, cFileDescriptor, "<init>", "()V");
    jfieldID descriptorID = (*env)->GetFieldID(env, cFileDescriptor, "descriptor", "I");
    mFileDescriptor = (*env)->NewObject(env, cFileDescriptor, iFileDescriptor);
    (*env)->SetIntField(env, mFileDescriptor, descriptorID, (jint) portFD);

    glbPortNum = portNum;

    return mFileDescriptor;
}

/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    close
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_newpos_uart_SerialPort_close
(JNIEnv *env, jclass thiz)
{
    return portClose(glbPortNum);
}

/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    injectKey
 * Signature: (IIII[B)I
 */
JNIEXPORT jint JNICALL Java_com_newpos_uart_SerialPort_injectKey
  (JNIEnv *env, jclass thiz, jint ks, jint keyType, jint keyIndex, jint keyLen, jbyteArray key)
  {
    int ret = -1 ;
    unsigned char keyData[keyLen];
    
    (*env)->GetByteArrayRegion(env,key,0,keyLen,keyData);
    
    ret = ped_inject_key(ks,keyType,keyIndex,keyLen,keyData);
    if (ret < 0) {
        LOGE("injectKey failed, error:%s",strerror(errno));
        return errno;
    }    
    return ret;
  }

/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    writeKeyUnified
 * Signature: (IIIIII[B)I
 */
JNIEXPORT jint JNICALL Java_com_newpos_uart_SerialPort_writeKeyUnified
  (JNIEnv *env, jclass thiz, jint ks, jint keyType, jint masterKeyIndex, jint destKeyIndex, jint verifyMode, jint keyLen, jbyteArray keyData)
  {
    if(keyData == NULL) {
        LOGE("writeKeyUnified,invalid parameters");
        return JNI_ERR;
    }
   
    int ret = -1;
    unsigned char keyContent[keyLen];
    (*env)->GetByteArrayRegion(env,keyData,0,keyLen,keyContent);
  
    ret = ped_write_key_unified(ks,keyType,masterKeyIndex,destKeyIndex,verifyMode,keyLen,keyContent);
    if (ret < 0) {
        LOGE("writeKeyUnified failed, error:%s",strerror(errno));
        return errno;
    }
    return ret;      
  }  
  
/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    vendorGet
 * Signature: ([B[B[I)I
 */
JNIEXPORT jint JNICALL Java_com_newpos_uart_SerialPort_vendorGet
  (JNIEnv *env, jclass thiz, jbyteArray key, jbyteArray value, jintArray len)
  {
    int ret = -1;
    
    int key_len = (*env)->GetArrayLength(env, key);
    unsigned char keyChar[key_len + 1];
    (*env)->GetByteArrayRegion(env, key, 0, key_len, keyChar);
    keyChar[key_len] = 0x00;
    
    char buff[256];
    int valuelen;
    ret = terminal_get_hw_config(keyChar, buff, sizeof(buff), &valuelen);
    if(ret != 0){
        LOGE("vendorGet failed, ret: %d", ret);
        return ret;
    }
    jbyte *valuebyte = (jbyte *) buff;
    (*env)->SetByteArrayRegion(env, value, 0, valuelen, valuebyte);
    jint valueLenArray[1] = {valuelen};
    (*env)->SetIntArrayRegion(env, len, 0, 1, valueLenArray);
    
    return ret;  
  }

/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    checkKeyUnified
 * Signature: (III)I
 */
JNIEXPORT jint JNICALL Java_com_newpos_uart_SerialPort_checkKeyUnified
  (JNIEnv *env, jclass thiz, jint ks, jint keyType, jint keyIndex)
  {
    int ret = -1;

    ret = ped_check_key_unified(ks,keyType,keyIndex,0);
    if (ret < 0) {
        LOGE("checkKeyUnified failed, error:%s",strerror(errno));
        return errno;
    }    
    
    return ret; 
  }  
/*
 * Class:     com_newpos_uart_SerialPort
 * Method:    sysBeep
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_com_newpos_uart_SerialPort_sysBeep
  (JNIEnv *env, jclass thiz, jint frequency, jint keepms){
    beep(frequency,keepms);
}



