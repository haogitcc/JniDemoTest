//#include <jni.h>
#include <string>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "com_serialport_SerialPort.h"

#include <android/log.h>
static const char *TAG="nativeSerial";
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__)
//#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG , TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO , TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN , TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR , TAG, __VA_ARGS__)

#define SUCCESS 1;
#define FAILED 0;

int fd;

static speed_t getBaudrate(jint baudrate)
{
    switch(baudrate) {
        case 0: return B0;
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return -1;
    }
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeOpen
 * Signature: (Ljava/lang/String;I)Ljava/io/FileDescriptor;
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeOpen
        (JNIEnv *env, jobject obj, jstring path, jint baudrate)
{

    speed_t speed;
    jobject mFileDescriptor;

    /* Check arguments */
    {
        speed = getBaudrate(baudrate);
        if (speed == -1) {
            /* TODO: throw an exception */
            LOGE("Invalid baudrate");
            return NULL;
        }
    }

    /* Opening device */
    {
        jboolean iscopy;
        const char *path_utf = env->GetStringUTFChars(path, &iscopy);
        LOGE("Opening serial port %s with flags 0x%x", path_utf, O_RDWR );
        fd = open(path_utf, O_RDWR );
        LOGE("open() fd = %d", fd);
        env->ReleaseStringUTFChars( path, path_utf);
        if (fd == -1)
        {
            /* Throw an exception */
            LOGE("Cannot open port");
            /* TODO: throw an exception */
            return FAILED;
        }
    }

    /* Configure device */
    {
        struct termios cfg;
        LOGE("Configuring serial port");
        if (tcgetattr(fd, &cfg))
        {
            LOGE("tcgetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return FAILED;
        }

        cfmakeraw(&cfg);
        cfsetispeed(&cfg, speed);
        cfsetospeed(&cfg, speed);

        if (tcsetattr(fd, TCSANOW, &cfg))
        {
            LOGE("tcsetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return FAILED;
        }
    }

    return SUCCESS;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeSendBytes
 * Signature: (I[BII)I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeSendBytes
        (JNIEnv *env, jobject obj, jint length, jbyteArray message, jint offset, jint timeousMs)
{
    int ret;
    jbyte *messageBytes;

    messageBytes = (*env).GetByteArrayElements( message, NULL);
    if (NULL == messageBytes)
    {
        return 0; /* exception already thrown */
    }

    do
    {
        ret = write(fd, messageBytes, length);
        if (ret == -1)
        {
            if (ENXIO == errno)
            {
                return ret;
            }
            else
            {
                return ret;
            }
        }
        length -= ret;
        message += ret;
    }
    while (length > 0);
    (*env).ReleaseByteArrayElements( message, messageBytes, 0);
    return ret;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeReceiveBytes
 * Signature: (I[BII)I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeReceiveBytes
        (JNIEnv *env, jobject obj, jint length, jbyteArray message, jint offset, jint timeoutMs)
{
    int ret;
    jbyte *messageBytes;
    messageBytes = (*env).GetByteArrayElements( message, NULL);
    if (NULL == messageBytes)
    {
        return 0; /* exception already thrown */
    }
    uint32_t messageLength;
    fd_set set;
    int status = 0;
    struct timeval tv;

    messageLength = 0;

    do
    {
        FD_ZERO(&set);
        FD_SET(fd, &set);
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        /* Ideally should reset this timeout value every time through */
        ret = select(fd + 1, &set, NULL, NULL, &tv);
        if (ret < 1)
        {
            return FAILED;
        }
        ret = read(fd, message, length);
        if (ret == -1)
        {
            if (ENXIO == errno)
            {
                return FAILED;
            }
            else
            {
                return FAILED;
            }
        }

        if (0 == ret)
        {
            /**
             * We should not be here, coming here means the select()
             * is success , but we are not able to read the data.
             * check the serial port connection status.
             **/
            ret = ioctl(fd, TIOCMGET, &status);
            if (-1 == ret)
            {
                /* not success. check for errno */
                if (EIO == errno)
                {
                    /**
                     * EIO means I/O error, may serial port got disconnected,
                     * throw the error.
                     **/
                    return FAILED;
                }
            }
        }

        length -= ret;
        messageLength += ret;
        message += ret;
    }
    while (length > 0);

    (*env).ReleaseByteArrayElements( message, messageBytes, 0);

    return ret;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeSetBaudRate
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeSetBaudRate
        (JNIEnv *, jobject, jint rate)
{
    struct termios t;

    tcgetattr(fd, &t);

#define BCASE(t,n) case n: cfsetispeed((t),B##n); cfsetospeed((t),B##n); break;
    switch (rate)
    {
        BCASE(&t, 9600);
        BCASE(&t, 19200);
        BCASE(&t, 38400);
        // Believe it or not, speeds beyond 38400 aren't required by POSIX.
#ifdef B57600
        BCASE(&t, 57600);
#endif
#ifdef B115200
        BCASE(&t, 115200);
#endif
#ifdef B230400
        BCASE(&t, 230400);
#endif
#ifdef B460800
        BCASE(&t, 460800);
#endif
#ifdef B921600
        BCASE(&t, 921600);
#endif
        default:
            return FAILED;
    }
#undef BCASE
    if (tcsetattr(fd, TCSANOW, &t) != 0)
    {
        return FAILED;
    }
    return SUCCESS;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeGetBaudRate
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeGetBaudRate
        (JNIEnv *, jobject)
{
    return 0xFFFF;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeClose
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeClose
        (JNIEnv *, jobject)
{
    close(fd);
    return SUCCESS;
}

/*
 * Class:     com_serialport_SerialPort
 * Method:    nativeFlush
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeFlush
        (JNIEnv *, jobject)
{
    if (tcflush(fd, TCOFLUSH) == -1)
    {
        return FAILED;
    }
    return SUCCESS;
}
