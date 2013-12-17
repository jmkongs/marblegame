/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.accelerometerplay;

import android.app.Activity;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;
/**
 * This is an example of using the accelerometer to integrate the device's
 * acceleration to a position using the Verlet method. This is illustrated with
 * a very simple particle system comprised of a few iron balls freely moving on
 * an inclined wooden table. The inclination of the virtual table is controlled
 * by the device's accelerometer.
 * 
 * @see SensorManager
 * @see SensorEvent
 * @see Sensor
 */

public class AccelerometerPlayActivity extends Activity {

	public static AccelerometerPlayActivity apa;
	
    private SimulationView mSimulationView;

    private PowerManager mPowerManager;
    private WindowManager mWindowManager;
    private Display mDisplay;
    private WakeLock mWakeLock;
    public TimingService boundService;

    @Override
    public void onDestroy()
    {
    	doUnbindService();
    	super.onDestroy();
    }
    
    private ServiceConnection mConnection = new ServiceConnection()
    {
        public void onServiceConnected(ComponentName className, IBinder service)
        {
            // This is called when the connection with the service has been
            // established, giving us the service object we can use to
            // interact with the service.  Because we have bound to a explicit
            // service that we know is running in our own process, we can
            // cast its IBinder to a concrete class and directly access it.
            boundService = ((TimingService.LocalBinder)service).getService();
        }

        public void onServiceDisconnected(ComponentName className)
        {
            // This is called when the connection with the service has been
            // unexpectedly disconnected -- that is, its process crashed.
            // Because it is running in our same process, we should never
            // see this happen.
            boundService = null;
        }
    };
    boolean mIsBound;
    
    void doBindService()
    {
        // Establish a connection with the service.  We use an explicit
        // class name because there is no reason to be able to let other
        // applications replace our component.
    	try
    	{
        bindService(new Intent(this, 
                TimingService.class), mConnection, Context.BIND_AUTO_CREATE);
    	}
    	catch( SecurityException e)
    	{
    		Log.e(ACTIVITY_SERVICE, e.toString());
    	}
        mIsBound = true;
    }

    void doUnbindService()
    {
        if (mIsBound) {
            // Detach our existing connection.
            unbindService(mConnection);
            mIsBound = false;
        }
    }
    
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
apa = this;
        // Get an instance of the PowerManager
        mPowerManager = (PowerManager) getSystemService(POWER_SERVICE);

        // Get an instance of the WindowManager
        mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        mDisplay = mWindowManager.getDefaultDisplay();

        // Create a bright wake lock
        mWakeLock = mPowerManager.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, getClass()
                .getName());

        // instantiate our simulation view and set it as the activity's content
        mSimulationView = new SimulationView(this);
        setContentView(mSimulationView);
        
        doBindService();
    }

    @Override
    protected void onResume() {
        super.onResume();
        /*
         * when the activity is resumed, we acquire a wake-lock so that the
         * screen stays on, since the user will likely not be fiddling with the
         * screen or buttons.
         */
        mWakeLock.acquire();

        // Start the simulation
        mSimulationView.startSimulation();
    }

    @Override
    protected void onPause() {
        super.onPause();
        /*
         * When the activity is paused, we make sure to stop the simulation,
         * release our sensor resources and wake locks
         */

        // Stop the simulation
        mSimulationView.stopSimulation();

        // and release our wake-lock
        mWakeLock.release();
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
    	// Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.accelerometer_play, menu);
        return super.onCreateOptionsMenu(menu);
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle presses on the action bar items
        switch (item.getItemId()) {
            case R.id.action_reload:
            	finish();
            	startActivity(getIntent());
                return true;
            case R.id.action_exit:
                finish();
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    
    }
}
