package com.serialport;

public class ReadCallback {

    public int len;
    public String strMsg;

    public ReadCallback(int len, String strMsg) {
        this.len = len;
        this.strMsg = strMsg;
    }
}
