//#include <jni.h>

#include "com_serialport_SerialPort.h"

#include <android/log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <malloc.h>
#include <fcntl.h>              /* open */
#include <assert.h>
#include <pthread.h>


static const char *TAG="nativeSerial";
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG , TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO , TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN , TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR , TAG, __VA_ARGS__)

#define FALSE  -1
#define TRUE   0

static int serial_fd = -1;

// Function for ReadCallback
#define EV_READ_FD 1
#define EV_WRITE_FD 2
bool reading = false;
JavaVM *hover_jvm;

typedef struct
{
    jobject o_read_listener;
    jmethodID m_read_receive;

    jclass z_read_callback;
    jmethodID m_read_callback_init;
} jni_callback_t;

// 定义承载 Java-Listener 和 Java-Callback 实例和 methodID 的结构体实例
static jni_callback_t *jni_callback;

// 动态注册 native 实现函数
static const char *CLASS_NAME = "com/serialport/SerialPort";//类名

static JNINativeMethod method[] = {//本地方法描述
        "nativeAddReadListener",//Java方法名
        "(Lcom/serialport/ReadListener;)V",//方法签名
        (void *) Java_com_serialport_SerialPort_nativeAddReadListener //绑定本地函数
};

void hover_attach_jni_env(JNIEnv **pEnv);
static bool bindNative(JNIEnv *env);
void *read_async_thread_func(void *args);
int SercdSelect(int* dev_fd, long PollInterval);

// Function for Serial
int printer_Open(const char* port);
void printer_Close(int fd);
int printer_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity);
int printer_SetBaudRate(int fd, int speed);
static int printer_Init(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity);
static int printer_Send(int fd, char *send_buf, int data_len);
ssize_t WriteToDev(int fd, const void *buf, size_t count);
ssize_t ReadFromDev(int fd, void *buf, size_t count);

/*******************************************************************
 ****************************Serial function****************************
*******************************************************************/
int printer_Open(const char* port)
{
    int fd = -1;
    int flags = O_RDWR | O_NOCTTY;// | O_NONBLOCK | O_NDELAY |O_SYNC;
    LOGD("Opening serial port %s with flags 0x%x", port, flags);
//    fd = open(port, flags);
    fd = open(port, flags, 0);
    if (FALSE == fd)
    {
        LOGD("Can't Open Serial Port! %s\n", strerror(errno));
    }

    return fd;
}

void printer_Close(int fd)
{
    if (close(fd) == FALSE)
    {
        LOGE("Close Serial Port! %s\n", strerror(errno));
    }
}

/*******************************************************************
* fd
* speed     串口速度
* flow_ctrl   数据流控制
* databits   数据位   取值为 7 或者8
* stopbits   停止位   取值为 1 或者2
* parity     效验类型 取值为N,E,O,,S
*
*return:
*正确返回为1，错误返回为0
*******************************************************************/
int printer_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity)
{
    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};
    int   speed_size;
    struct termios oldoptions, options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,
    该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if ( tcgetattr( fd,&oldoptions)  !=  0)
    {
        LOGD("get serial options error!%s\n", strerror(errno));
        return(FALSE);
    }

    memset(&options, 0, sizeof(options));

    speed_size = sizeof(speed_arr) / sizeof(int);
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < speed_size;  i++)
    {
        if  (speed == name_arr[i])
        {
            //LOGD("speed=%d\n", name_arr[i]);
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
            break;
        }
    }

    if(i >= speed_size){
        LOGD("Unsupported baudrate!\n");
        return FALSE;
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {
        case 0 ://不使用流控制
            //LOGD("flow_ctrl=%d\n", flow_ctrl);
            options.c_cflag &= ~CRTSCTS;
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;

        case 2 ://使用软件流控制
            options.c_iflag |= (IXON | IXOFF | IXANY);
            break;

        default://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
    }

    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
            options.c_cflag |= CS5;
            break;
        case 6    :
            options.c_cflag |= CS6;
            break;
        case 7    :
            options.c_cflag |= CS7;
            break;
        case 8:
            //LOGD("databits=%d\n", databits);
            options.c_cflag |= CS8;
            break;
        default:
            LOGD("Unsupported data size\n");
            return (FALSE);
    }

    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            //LOGD("parity=%c\n", parity);
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= (INPCK | ISTRIP);
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            LOGD("Unsupported parity\n");
            return (FALSE);
    }

    // 设置停止位
    switch (stopbits)
    {
        case 1:
            //LOGD("stopbits=%d\n", stopbits);
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            LOGD("Unsupported stop bits\n");
            return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 0;//1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;//1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        LOGD("com set error!%s\n", strerror(errno));
        return (FALSE);
    }

    return (TRUE);
}

int printer_SetBaudRate(int fd, int speed)
{
    struct termios oldoptions, options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,
    该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if ( tcgetattr( fd,&oldoptions)  !=  0)
    {
        LOGD("get serial options error!%s\n", strerror(errno));
        return(FALSE);
    }

    memset(&options, 0, sizeof(options));

#define BCASE(t,n) case n: cfsetispeed((t),B##n); cfsetospeed((t),B##n); break;
    switch (speed)
    {
        BCASE(&options, 9600);
        BCASE(&options, 19200);
        BCASE(&options, 38400);
        // Believe it or not, speeds beyond 38400 aren't required by POSIX.
#ifdef B57600
        BCASE(&options, 57600);
#endif
#ifdef B115200
        BCASE(&options, 115200);
#endif
#ifdef B230400
        BCASE(&options, 230400);
#endif
#ifdef B460800
        BCASE(&options, 460800);
#endif
#ifdef B921600
        BCASE(&options, 921600);
#endif
        default:
            LOGD("Unsupported baudrate!\n");
            return (FALSE);
    }
#undef BCASE
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0)
    {
        return (FALSE);
    }

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        LOGE("com set error!%s\n", strerror(errno));
        return (FALSE);
    }

    return (TRUE);
}

/*******************************************************************
* 入口参数：        fd       :  文件描述符
*     speed  :  串口速度
*     flow_ctrl  数据流控制
*     databits   数据位   取值为 7 或者8
*     stopbits   停止位   取值为 1 或者2
*     parity     效验类型 取值为N,E,O,,S
*
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int printer_Init(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity)
{
    int err;

    //设置串口数据帧格式
    err = printer_Set(fd, speed, flow_ctrl, databits, stopbits, parity);

    return err;
}

int printer_Send(int fd, char *send_buf, int data_len)
{
    int len = 0;
//    LOGE("printer_Send --> %s", send_buf);
    len = write(fd, send_buf, data_len);
//    LOGE("printer_Send %d", len);
    if (len == data_len )
    {
        return len;
    }else{
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}

ssize_t WriteToDev(int fd, const void *buf, size_t count)
{
    return write(fd, buf, count);
}

ssize_t ReadFromDev(int fd, void *buf, size_t count)
{
    return read(fd, buf, count);
}

/************************************
 * For Callback Function
 */
static bool bindNative(JNIEnv *env) {
//    LOGE("bindNative");
    jclass clazz;
    clazz = env->FindClass(CLASS_NAME);
    if (clazz == NULL) {
        return false;
    }
    return env->RegisterNatives(clazz, method, sizeof(method) / sizeof(method[0])) == 0;
}

jint JNI_OnLoad(JavaVM *vm, void *reserved)
{
//    LOGE("JNI_OnLoad");
    JNIEnv *env = NULL;
    if (vm->GetEnv((void **)&env, JNI_VERSION_1_6) != JNI_OK)
        return -1;
    assert(env != NULL);
    if (!bindNative(env))
        return -1;

    hover_jvm = vm;
    return JNI_VERSION_1_6;
}

void hover_attach_jni_env(JNIEnv **pEnv) {
    hover_jvm->AttachCurrentThread(pEnv, NULL);
}

int SercdSelect(int* dev_fd, long PollInterval)
{
    fd_set InFdSet;
    fd_set OutFdSet;
    int highest_fd = -1, selret;
    struct timeval BTimeout;
    struct timeval newpoll;
    int ret = 0;

    FD_ZERO(&InFdSet);

    if (dev_fd) {
        FD_SET(*dev_fd, &OutFdSet);
        highest_fd = *dev_fd + 1;
    }

    if (dev_fd) {
        FD_SET(*dev_fd, &InFdSet);
        highest_fd = *dev_fd + 1;
    }

    BTimeout.tv_sec = PollInterval / 1000;
    BTimeout.tv_usec = (PollInterval % 1000) * 1000;

    selret = select(highest_fd + 1, &InFdSet, &OutFdSet, NULL, &BTimeout);

    if (selret < 0)
        ret =selret;

    if (*dev_fd && FD_ISSET(*dev_fd, &InFdSet)) {
        ret |= EV_READ_FD;
    }

    if (*dev_fd && FD_ISSET(*dev_fd, &OutFdSet)) {
        ret |= EV_WRITE_FD;
    }

    return ret;
}

void *read_async_thread_func(void *args)
{
//    LOGE("read_async_thread_func");
    // JNI_OnLoad 时保存 jvm，这里使用 jvm->AttachCurrentThread 获取 JNIEnv，暂不做详细介绍.
    JNIEnv *env = NULL;
    hover_attach_jni_env(&env);

    while (reading)
    {
        if(serial_fd < 0)
            continue;
        int selret = SercdSelect(&serial_fd, 10);
        if (selret & EV_READ_FD)
        {
            // start read
            /* Chars read */
            char readbuf[1024];
            int readLen;
            readLen= read(serial_fd, readbuf, sizeof(readbuf));
            if(readLen > 0) {
//                LOGE("<-- nativeReceiveBytes len=%d, buf=%s", readLen, readbuf);
                char buf[readLen];
                strncpy(buf, readbuf, sizeof(buf));
                buf[readLen] = '\0';

                // 使用 java 构造函数生成 ReadCallback 的实例
                int len = readLen;
                jstring strMsg = env->NewStringUTF(buf);
                jobject read_callback = env->NewObject(jni_callback->z_read_callback, jni_callback->m_read_callback_init, len, strMsg);

                // 调用 GreetListener 的 onGreetReceive 函数，完成调用流程.
                env->CallVoidMethod(jni_callback->o_read_listener, jni_callback->m_read_receive, read_callback);

                // 销毁局部引用
                env->DeleteLocalRef(strMsg);
                env->DeleteLocalRef(read_callback);
                strMsg = NULL;
                read_callback = NULL;
            }
        }
        if (selret & EV_WRITE_FD)
        {
//            LOGE("Write");
        }
    }

    LOGE("stopReading");

    // 销毁全局引用 --- 如果有多处或多次回调，自行判断销毁时机.
    env->DeleteGlobalRef(jni_callback->o_read_listener);
    env->DeleteGlobalRef(jni_callback->z_read_callback);
    jni_callback->o_read_listener = NULL;
    jni_callback->z_read_callback = NULL;
    free(jni_callback);
    jni_callback = NULL;

    return 0;
}

/*******************************************
 * JNI Function
 */
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeOpenPb
        (JNIEnv *env, jobject, jstring path)
{
    /* Opening device */
    int ret;
    jboolean iscopy;
    const char *path_utf = env->GetStringUTFChars(path, &iscopy);

    serial_fd = printer_Open(path_utf);
    LOGD("openpb fd = %d", serial_fd);
    if(serial_fd == FALSE)
    {
        //if open error,signal to unclock acmsts_cond_t;
        LOGD("open %s error!\n", path_utf);
        ret = FALSE;
    }
    else
    {
        ret = TRUE;
    }
    env->ReleaseStringUTFChars( path, path_utf);
//    if(ret == TRUE)
//        LOGE("open done");
    return ret;
}
JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeOpen
        (JNIEnv *env, jobject obj, jstring path, jint baudrate)
{
    /* Opening device */
    int ret;
    jboolean iscopy;
    const char *path_utf = env->GetStringUTFChars(path, &iscopy);

    serial_fd = printer_Open(path_utf);
    LOGD("open() fd = %d", serial_fd);
    if(serial_fd == FALSE)
    {
        //if open error,signal to unclock acmsts_cond_t;
        LOGD("open %s error!\n", path_utf);
        ret = FALSE;
    }
    else
    {
        //for tty setup
        int speed = baudrate;
        int flow_ctrl = 0;
        int databits = 8;
        int stopbits = 1;
        char parity = 'N';
        int init_ret;

        //open ttyacm ok, to init it.
        init_ret = printer_Init(serial_fd, speed, flow_ctrl, databits, stopbits, parity);
//        LOGE("init success");
        if(init_ret == TRUE)
        {
            //init ok
            LOGE("baudrate=%d, flow_ctrl=%d, databits=%d, stopbits=%d, parity=%c",
                    speed,
                    flow_ctrl,
                    databits,
                    stopbits,
                    parity);
//            LOGE("Set Port Exactly! serial_fd=%d", serial_fd);
            ret = TRUE;
        }
        else
        {
            printer_Close(serial_fd);
            ret = FALSE;
        }
    }
    env->ReleaseStringUTFChars( path, path_utf);
//    if(ret == TRUE)
//        LOGE("open done");
    return ret;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeSetBaudRate
        (JNIEnv *env, jobject obj, jint rate)
{
    printer_SetBaudRate(serial_fd, rate);
    return TRUE;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeGetBaudRate
        (JNIEnv *env, jobject obj)
{
    return 0xFFFF;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeClose
        (JNIEnv *env, jobject obj)
{
    printer_Close(serial_fd);
    return TRUE;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeFlush
        (JNIEnv *env, jobject obj)
{
    if (tcflush(serial_fd, TCOFLUSH) == -1)
    {
        return FALSE;
    }
    return TRUE;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeSendBytes
        (JNIEnv *env, jobject obj, jint length, jbyteArray message, jint offset, jint timeousMs)
{
    jboolean isCopy;
    jbyte* messgaeBytes = env->GetByteArrayElements(message, &isCopy);

    int sendLen;
    sendLen = printer_Send(serial_fd, reinterpret_cast<char *>(messgaeBytes), length);
//    LOGE("nativeSendBytes --> len=%d, buf=%s", sendLen, buf);
    env->ReleaseByteArrayElements(message, messgaeBytes, 0);
    return sendLen;
}

JNIEXPORT jint JNICALL Java_com_serialport_SerialPort_nativeReceiveBytes
        (JNIEnv *env, jobject obj, jint length, jbyteArray message, jint offset, jint timeoutMs)
{
    jboolean isCopy;
    jbyte* messgaeBytes = env->GetByteArrayElements(message, &isCopy);

    int readLen;
    readLen= ReadFromDev(serial_fd, reinterpret_cast<char *>(messgaeBytes), length);
//    if(readLen > 0)
//        LOGE("<-- nativeReceiveBytes len=%d, buf=%s", readLen, buf);
    env->ReleaseByteArrayElements( message, messgaeBytes, 0);
    return readLen;
}


// JNI Callback
/**
 * jni 动态注册对应的 native 实现函数。
 * 创建 Java-Listener 和 Java-Callback 的实例和 methodID 并赋值给 jni_callback_t.
 */
JNIEXPORT void JNICALL
Java_com_serialport_SerialPort_nativeAddReadListener(JNIEnv *env, jobject thiz,
                                                    jobject read_listener) {
//    LOGE("Call JNI nativAddReadListener");
    if(!read_listener)
        return;

    // 获得回调接口 函数onDataReceive 的 methodID.
    jclass clazz_read_listener = env->GetObjectClass(read_listener);
    jmethodID method_read_receive = env->GetMethodID(clazz_read_listener, "onDataReceive", "(Lcom/serialport/ReadCallback;)V");

    // 获得自定义 Callback类 ReadCallback 构造函数的 methodID.
    jclass clazz_read_callback = env->FindClass("com/serialport/ReadCallback");
    jmethodID method_read_callback_init = env->GetMethodID(clazz_read_callback, "<init>", "(ILjava/lang/String;)V");

    // 这里创建 jni_callback_t 的实例，创建 Listener 和 Callback 的全局引用并赋值.
    jni_callback = (jni_callback_t *)malloc(sizeof(jni_callback_t));
    memset(jni_callback, 0, sizeof(jni_callback_t));
    jni_callback->o_read_listener = env->NewGlobalRef(read_listener);
    jni_callback->m_read_receive = method_read_receive;
    jni_callback->z_read_callback = (jclass)env->NewGlobalRef(clazz_read_callback);
    jni_callback->m_read_callback_init = method_read_callback_init;

    // 销毁局部引用
    env->DeleteLocalRef(clazz_read_listener);
    env->DeleteLocalRef(clazz_read_callback);
    clazz_read_listener = NULL;
    clazz_read_callback = NULL;

    // 这里创建子线程模拟异步回调
    reading = true;
    pthread_t ntid;
    pthread_create(&ntid, NULL, read_async_thread_func, NULL);
}

JNIEXPORT void JNICALL
Java_com_serialport_SerialPort_nativeRemoveReadListener(JNIEnv *env, jobject thiz)
{
    reading = false;
}






