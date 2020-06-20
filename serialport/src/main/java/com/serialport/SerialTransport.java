package com.serialport;

public interface SerialTransport
{
  void open(String port, int baudrate) throws Exception;

  void sendBytes(int length, byte[] message, int offset, int timeoutMs) throws Exception;

  byte[] receiveBytes(int length, byte[] messageSpace, int offset, int timeoutMillis);

  int getBaudRate();

  void setBaudRate(int rate);

  void flush();

  void close();  
 
}

