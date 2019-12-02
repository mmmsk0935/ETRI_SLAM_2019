package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.icu.text.SimpleDateFormat;
import android.os.Environment;
import android.os.Handler;
import android.view.MotionEvent;

import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;

public class MainActivity extends AppCompatActivity {

    private SensorManager mSensorManager;
    private Sensor mPose6DoF = null;
    private Sensor mRotationSensor = null;
    private Sensor mAccSensor = null;
    private Sensor mGyroSensor = null;
    private SensorEventListener mSensorEventListner;

    private double qX;
    private double qY;
    private double qZ;
    private double qW;

    private String strQX;
    private String strQY;
    private String strQZ;
    private String strQW;

    private String foldername;
    private String filename;
    private String timeString;

    File fDir = null;
    FileOutputStream fosFileOutputStream = null;
    BufferedWriter bwBufferedWriter = null;
    SimpleDateFormat sdfCurrentTime = null;
    SimpleDateFormat sdfCurrentTimeSpace = null;
    Date dCurrentTime;

    TextView textViewQX;
    TextView textViewQY;
    TextView textViewQZ;
    TextView textViewQW;
    TextView textViewFolderF;

    Handler mHandler = null;
    Runnable mTask = null;

    public static boolean checkAvailable() {

        // Retrieving the external storage state
        String state = Environment.getExternalStorageState();

        // Check if available
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    public static boolean checkWritable()
    {
        // Retrieving the external storage state
        String state = Environment.getExternalStorageState();

        // Check if writable
        if (Environment.MEDIA_MOUNTED.equals(state) )
        {
            if(Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
                return false;
            }else{
                return true;
            }
        }
        return false;
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        //mAccSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        //mGyroSensor= mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        mSensorEventListner = new RotationListener();

        findViewById(R.id.IMUbutton).setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch( View v, MotionEvent event)
            {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        break;
                    case MotionEvent.ACTION_UP:

                        if( bwBufferedWriter != null )
                            break;

                        foldername = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Recording";

                        fDir = new File(foldername);

                        if (!fDir.exists()) {
                            fDir.mkdir();
                        }
                        //ActivityCompat.requestPermissions();
                        dCurrentTime = new Date();
                        sdfCurrentTime = new SimpleDateFormat("MM_dd_HH_mm_ss_SS");
                        sdfCurrentTimeSpace = new SimpleDateFormat("MM\tdd\tHH\tmm\tss\tSSS");
                        filename = "IMU_orientation_" + sdfCurrentTime.format(dCurrentTime) + ".txt";

                        try {
                            bwBufferedWriter = new BufferedWriter(new FileWriter(foldername+"/"+filename, true));
                        }
                        catch(IOException ex)
                        {
                        }

                        mSensorManager.registerListener(mSensorEventListner, mRotationSensor, mSensorManager.SENSOR_DELAY_UI);
                        //mSensorManager.unregisterListener(mSensorEventListner);
                        break;

                }
                return false;
            }

        });

        findViewById(R.id.CloseButton).setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch( View v, MotionEvent event)
            {
                switch(event.getAction())
                {
                    case MotionEvent.ACTION_DOWN:
                        //mSensorManager.registerListener(mSensorEventListner, mRotationSensor, mSensorManager.SENSOR_DELAY_UI);
                        break;
                    case MotionEvent.ACTION_UP:
                        try {
                            bwBufferedWriter.flush();
                            bwBufferedWriter.close();
                            bwBufferedWriter = null;
                        }
                        catch(Exception e) {
                        }

                        mSensorManager.unregisterListener(mSensorEventListner);
                        break;

                }
                return false;
            }

        });

    }

    @Override
    public void onPause(){
        super.onPause();
        //Log.e("LOG", "onPause()");
        //mSensorManager.unregisterListener(mSensorEventListner);
    }

    @Override
    public void onDestroy(){

        //Log.e("LOG", "onDestroy()");
        mSensorManager.unregisterListener(mSensorEventListner);
        try {
            bwBufferedWriter.flush();
            bwBufferedWriter.close();
        }
        catch(Exception e) {
        }

        super.onDestroy();

    }

    private class RotationListener implements SensorEventListener {

        @Override
        public void onSensorChanged(SensorEvent event) {
            qX = event.values[0];
            qY = event.values[1];
            qZ = event.values[2];
            qW = event.values[3];

            strQX = Double.toString(qX);
            strQY = Double.toString(qY);
            strQZ = Double.toString(qZ);
            strQW = Double.toString(qW);

            textViewQX = findViewById(R.id.textViewIMUquatX);
            textViewQX.setText( Double.toString(qX) );

            textViewQY = findViewById(R.id.textViewIMUquatY);
            textViewQY.setText( Double.toString(qY) );

            textViewQZ = findViewById(R.id.textViewIMUquatZ);
            textViewQZ.setText( Double.toString(qZ) );

            textViewQW = findViewById(R.id.textViewIMUquatW);
            textViewQW.setText( Double.toString(qW) );

            textViewFolderF = findViewById(R.id.textViewFolder);
            textViewFolderF.setText(filename);

           try {
               dCurrentTime = new Date();
               timeString = sdfCurrentTimeSpace.format(dCurrentTime) +"\t";

               bwBufferedWriter.write( timeString );
               bwBufferedWriter.write("\t"+strQX+"\t"+strQY+"\t"+strQZ+"\t"+strQW);
               //bwBufferedWriter.write("\t"+event.values[4]+"\t"+event.values[5]+"\t"+event.values[6]);
               //bwBufferedWriter.write("\t"+event.values[7]+"\t"+event.values[8]+"\t"+event.values[9]+"\t"+event.values[10]);
               //bwBufferedWriter.write("\t"+event.values[11]+"\t"+event.values[12]+"\t"+event.values[13]+"\t"+event.values[14]+"\t"+event.values[14]);
               bwBufferedWriter.write("\n");
               bwBufferedWriter.flush();

               //mHandler.postDelayed(this,30);
            }
            catch (IOException ex) {
                textViewFolderF.setText( "IO exception");
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }


    }
}

