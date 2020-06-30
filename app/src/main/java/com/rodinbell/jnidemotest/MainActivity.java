package com.rodinbell.jnidemotest;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;

import com.serialport.ReadCallback;
import com.serialport.ReadListener;
import com.serialport.SerialPort;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";
    SerialPort serialPort;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        serialPort = new SerialPort("/dev/ttyACM0", 115200);

        serialPort.startReading(new ReadListener() {
            @Override
            public void onDataReceive(ReadCallback read) {
                Log.d(TAG, "onDataReceive: " + read.strMsg);
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        serialPort.stopReading();
    }
}
