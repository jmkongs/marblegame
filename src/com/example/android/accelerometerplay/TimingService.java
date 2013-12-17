package com.example.android.accelerometerplay;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.Ringtone;
import android.media.RingtoneManager;
import android.net.Uri;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import android.view.Surface;
import android.widget.Toast;

    public class TimingService extends Service implements SensorEventListener {
 	   private int NOTIFICATION = 696969;
 	  private SensorManager mSensorManager;
 	   
 	 private Thread bgThread;
     private NotificationManager mNM;
 	/**
      * Class for clients to access.  Because we know this service always
      * runs in the same process as its clients, we don't need to deal with
      * IPC.
      */
 	public class LocalBinder extends Binder {
 		TimingService getService() {
 			return TimingService.this;
 		}
 	}
 	
     @Override
     public int onStartCommand(Intent intent, int flags, int startId) {
         
    	 

    	 
    	 return START_STICKY;
     }
     
     private long startTime = -1;   
     public void startTimer()
     {
    	 startTime = System.currentTimeMillis();
     }
     
     public long stopTimer()
     {
    	 return System.currentTimeMillis() - startTime;
     }
     
     @Override
     public void onCreate() {
         mNM = (NotificationManager)getSystemService(NOTIFICATION_SERVICE);
         
         // Display a notification about us starting.  We put an icon in the status bar.
         showNotification();
         
         // We want this service to continue running until it is explicitly
         // stopped, so return sticky.
        lastMove = System.currentTimeMillis(); 
    	 bgThread = performOnBackgroundThread( new Runnable() {
    
		@Override
		public void run() {
			// TODO Auto-generated method stub
			monitorAccelerometer();
		} } );
     }
     
     @Override
     public void onDestroy() {
         // Cancel the persistent notification.
         mNM.cancel(NOTIFICATION);

       	bgThread.interrupt();
       	mSensorManager.unregisterListener(this);
     }
 	
		@Override
		public IBinder onBind(Intent intent) {
			return mBinder;
		}
 	
	    // This is the object that receives interactions from clients.  See
	    // RemoteService for a more complete example.
	    private final IBinder mBinder = new LocalBinder();
	    
	    /**
	     * Show a notification while this service is running.
	     */
	    private void showNotification() {
	        // In this sample, we'll use the same text for the ticker and the expanded notification
	        CharSequence text = "Timing service started.";

	        // Set the icon, scrolling text and timestamp
	        Notification notification = new Notification(R.drawable.ball, text,
	                System.currentTimeMillis());

	        // The PendingIntent to launch our activity if the user selects this notification
	        PendingIntent contentIntent = PendingIntent.getActivity(this, 0,
	                new Intent(this, TimingService.class), 0);

	        // Set the info for the views that show in the notification panel.
	        notification.setLatestEventInfo(this, "Timing service label",
	                       text, contentIntent);

	        // Send the notification.
	        mNM.notify(NOTIFICATION, notification);
	    }
	    
	    private static Thread performOnBackgroundThread(final Runnable runnable) {
	        final Thread t = new Thread() {
	            @Override
	            public void run() {
	                try {
	                    runnable.run();
	                } finally {

	                }
	            }
	        };
	        t.start();
	        return t;
	    }
	    
	    public void monitorAccelerometer()
	    {
	    	try
	    	{
	    		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
	    		mSensorManager.registerListener(this, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_UI);
	    		}
	    	catch (Exception e)
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    }

		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
			// TODO Auto-generated method stub
			// What do?
		}

		private float prevX, prevY;
		private long lastMove = 0;
		private boolean warned = false;
		
		@Override
		public void onSensorChanged(SensorEvent event)
		{
			float x, y;
	        if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
	        {
	            return;
	        }
	        
	        x = event.values[0];
	        y = event.values[1];
	        
	        if( Math.abs(prevX - x) > 0.2f || Math.abs(prevY - y) > 0.2f )
	        {
	        	lastMove = System.currentTimeMillis();
	        	warned = false;
	        }
	        else if( lastMove != 0 )
	        {
	        	if( System.currentTimeMillis() - lastMove > 10000 && !warned )
	        	{
	        		warned = true;
	    	    	AccelerometerPlayActivity.apa.runOnUiThread( new Runnable() {	    	    		
	    	    		@Override
	    	    		public void run() {
	    	    			Toast.makeText(getBaseContext(), "Idle detected, will shutdown Marble Maze", Toast.LENGTH_SHORT).show();
	    	    		} });
	        	}
	        	else if (System.currentTimeMillis() - lastMove > 20000 )
	        	{
	        		Uri notification = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION);
	        		Ringtone r = RingtoneManager.getRingtone(getApplicationContext(), notification);
	        		r.play();
	        		mSensorManager.unregisterListener(this);
	        		AccelerometerPlayActivity.apa.finish();
	        	}
	        }
	        
        	prevX = x;
        	prevY = y;
		}
 }
