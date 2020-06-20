package com.rodinbell.jnidemotest;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.serialport.SerialPort;


public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    public void test(View view) {

        SerialPort serialPort = new SerialPort();
        try {
            serialPort.open("/dev/ttyACM0", 115200);
            Log.d(TAG, "test: connect success");

//            serialPort.sendBytes();
            
            serialPort.close();
            Log.d(TAG, "test: close success");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
