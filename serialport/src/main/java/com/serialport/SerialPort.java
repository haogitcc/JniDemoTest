package com.serialport;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class SerialPort implements SerialTransport{
    private static final String TAG = "SerialPort";

    private boolean opened=false;

    public SerialPort()
    {
        if(opened)
            return;
    }

    public SerialPort(String deviceName)
    {
        if(opened)
            return;
        try {
            openPb(deviceName);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public SerialPort(String deviceName, int baudrate)
    {
        try {
            open(deviceName, baudrate);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void openPb(String port) throws Exception {
        if (-1 == nativeOpenPb(port))
        {
            throw new Exception("Couldn't open device");
        }
        opened = true;
    }

    @Override
    public void open(String port, int baudrate) throws Exception {

        if (-1 == nativeOpen(port, baudrate))
        {
            throw new Exception("Couldn't open device");
        }
        opened = true;
    }

    @Override
    public void flush() {
        if (!opened)
        {
            return;
        }

        nativeFlush();
    }

    @Override
    public void close() {
        nativeClose();
    }

    public void setBaudRate(int rate)
    {
        nativeSetBaudRate(rate);
    }

    public int getBaudRate()
    {
        return nativeGetBaudRate();
    }

    public int Send(byte[] data)
    {
        int sendLen;
        sendLen = nativeSendBytes(data.length, data, 0, 10);
        System.err.println("SerialPort: Send " + sendLen);
        return sendLen;
    }

    public int Recv(byte[] recvBuf)
    {
        int recvLen;
        recvLen = nativeReceiveBytes(recvBuf.length, recvBuf, 0, 10);
        if(recvLen > 0)
            System.err.println("SerialPort: Recv " + recvLen);
        return recvLen;
    }

    public void startReading(ReadListener readListener)
    {
        nativeAddReadListener(readListener);
    }

    public void stopReading()
    {
        nativeRemoveReadListener();
    }

    // JNI
    private native int nativeOpenPb(String port);
    private native int nativeOpen(String port, int baudrate);
    private native int nativeSendBytes(int length, byte[] message, int offset, int timeoutMs);
    private native int nativeReceiveBytes(int length, byte[] message, int offset, int timeoutMs);
    private native int nativeSetBaudRate(int rate);
    private native int nativeGetBaudRate();
    private native int nativeClose();
    private native int nativeFlush();
    private static native void nativeAddReadListener(ReadListener readListener);
    private static native void nativeRemoveReadListener();
    static {
        System.loadLibrary("nativeSerial");
//        load();
    }

    /**
     * Todo: for load lib in jar
     */
    private static void load()
    {
        String libname ="";

        String osname = System.getProperty("os.name").toLowerCase();
        String osarch = System.getProperty("os.arch");
        if (osname.startsWith("mac os"))
        {
            osname = "mac";
            osarch = "universal";
        }
        if (osname.startsWith("windows"))
        {
            osname = "win";
            if (osarch.startsWith("a") && osarch.endsWith("64"))
            {
                osarch = "x64";
            }
        }
        if (osarch.startsWith("i") && osarch.endsWith("86"))
        {
            osarch = "x86";
        }

        if (osname.startsWith("linux"))
        {
            if(osarch.startsWith("mips"))
            {

            }
            if(osarch.startsWith("a"))
            {

            }
        }
        libname = osname + '-' + osarch + ".so";
        System.out.println("libname="+libname);

        libname = "libnativeSerial.so";
        System.out.println("libname="+libname);
        // try a bundled library
        try
        {
            InputStream in = SerialPort.class.getResourceAsStream(libname);
            if (in == null)
            {
                throw new RuntimeException("libname: "+libname+" not found");
            }
            File tmplib = File.createTempFile("libnativeSerial", ".so");
            tmplib.deleteOnExit();

            OutputStream out = new FileOutputStream(tmplib);
            byte[] buf = new byte[1024];
            for (int len; (len = in.read(buf)) != -1;)
            {
                out.write(buf, 0, len);
            }
            in.close();
            out.close();

            System.load(tmplib.getAbsolutePath());
        }
        catch (IOException e)
        {
            throw new RuntimeException("Error loading " + libname);
        }
    }
}