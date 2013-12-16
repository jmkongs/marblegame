package com.example.android.accelerometerplay;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.BitmapFactory.Options;
import android.graphics.RectF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.DisplayMetrics;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.Toast;

public class SimulationView extends View implements SensorEventListener
{
	private Context mContext;
    private Sensor mAccelerometer;

    private SensorManager mSensorManager;
    private float mXDpi;
    private float mYDpi;
    private float mMetersToPixelsX;
    private float mMetersToPixelsY;
    private Bitmap mBitmap;
    private Bitmap mWood;
    private float mSensorX;
    private float mSensorY;
    private long mSensorTimeStamp;
    private long mCpuTimeStamp;
    private Maze mMaze;
    private ParticleSystem mParticleSystem;
    private WindowManager windowManager;
    private Display mDisplay;
    DisplayMetrics mMetrics;
    private Particle mBall;
    
    private boolean timeStarted = false;
    
    private RectF startZone, endZone;
    
    public void startSimulation()
    {
        /* It is not necessary to get accelerometer events at a very high
         * rate, by using a slower rate (SENSOR_DELAY_UI), we get an
         * automatic low-pass filter, which "extracts" the gravity component
         * of the acceleration. As an added benefit, we use less power and
         * CPU resources.
         */
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
    }

    public void stopSimulation()
    {
        mSensorManager.unregisterListener(this);
    }
    
    public SimulationView(Context context)
    {
        super(context);
    	
        mMetrics = new DisplayMetrics();
        
        mContext = context;
        
        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) mContext.getSystemService(Context.SENSOR_SERVICE);
        windowManager = (WindowManager)mContext.getSystemService(Context.WINDOW_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mDisplay = windowManager.getDefaultDisplay();
        mDisplay.getMetrics(mMetrics);
        mXDpi = mMetrics.xdpi;
        mYDpi = mMetrics.ydpi;
        mMetersToPixelsX = mXDpi / 0.0254f;
        mMetersToPixelsY = mYDpi / 0.0254f;
        mParticleSystem = new ParticleSystem(0, 0);
        final int diameter = 16;
        mBall = new Particle(mMetersToPixelsX, mMetersToPixelsY, 0, 0, diameter );
        mParticleSystem.addParticle(mBall);
        // rescale the ball so it's about 0.5 cm on screen
        Bitmap ball = BitmapFactory.decodeResource(getResources(), R.drawable.ball);
        mBitmap = Bitmap.createScaledBitmap(ball, diameter, diameter, true);
        
        Options opts = new Options();
        opts.inDither = true;
        opts.inPreferredConfig = Bitmap.Config.RGB_565;
        mWood = BitmapFactory.decodeResource(getResources(), R.drawable.wood, opts);
        final int woodDstWidth = (int) (mMetrics.widthPixels);
        final int woodDstHeight = (int) (mMetrics.heightPixels);
        mWood = Bitmap.createScaledBitmap(mWood, woodDstWidth, woodDstHeight, true);
        mMaze = new Maze( 10, 15, mMetrics );
        mMaze.setStart(mBall);
        startZone = mMaze.getStartZone();
        endZone = mMaze.getEndZone();
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh)
    {
        final float x, y;
        x = mMaze.getCellSizeX();
        y = mMaze.getCellSizeY();
        mParticleSystem.setBounds(new RectF( 0 + x, 0 + y, w - x, h - y ) );
    }

    @Override
    public void onSensorChanged(SensorEvent event)
    {
        if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
        {
            return;
        }
        
        /* record the accelerometer data, the event's timestamp as well as
         * the current time. The latter is needed so we can calculate the
         * "present" time during rendering. In this application, we need to
         * take into account how the screen is rotated with respect to the
         * sensors (which always return data in a coordinate space aligned
         * to with the screen in its native orientation).
         */

        switch (mDisplay.getRotation())
        {
            case Surface.ROTATION_0:
                mSensorX = event.values[0];
                mSensorY = event.values[1];
                break;
                
            case Surface.ROTATION_90:
                mSensorX = -event.values[1];
                mSensorY = event.values[0];
                break;
            
            case Surface.ROTATION_180:
                mSensorX = -event.values[0];
                mSensorY = -event.values[1];
                break;
            
            case Surface.ROTATION_270:
                mSensorX = event.values[1];
                mSensorY = -event.values[0];
                break;
        }

        mSensorTimeStamp = event.timestamp;
        mCpuTimeStamp = System.nanoTime();
    }

    private Paint drawPaint = new Paint();
    
    @Override
    protected void onDraw(Canvas canvas)
    {
    	 // draw the background    
        canvas.drawBitmap(mWood, 0, 0, null);

        /* compute the new position of our object, based on accelerometer
         * data and present time.
         */
        final long now = mSensorTimeStamp + (System.nanoTime() - mCpuTimeStamp);
        final float sx = mSensorX;
        final float sy = mSensorY;

        mParticleSystem.update(sx, sy, now);
        
        mParticleSystem.checkForWallCollision(mMaze, canvas);
        
        RectF obj = new RectF();
        float x = mBall.getPosX();
        float y = mBall.getPosY();
        float r = mBall.getDiameter() /2.0f;
        obj.top = y - r;
        obj.right = x + r;
        obj.left = x - r;
        obj.bottom = y + r;
        
        if( !timeStarted )
        {

	        if( !obj.intersect(startZone) )
	        {
	        	// time start
	        	timeStarted = true;
	        	Toast.makeText(getContext(), "TIM EGO", Toast.LENGTH_LONG).show();
	        }
        }
        else
        {
        	if( obj.intersect(endZone))
        	{
        		// stop time, game over
        		Toast.makeText(getContext(), "YOU WIN 1000 internets", Toast.LENGTH_LONG).show();
        	}
        }
        
        drawPaint.setColor(Color.RED);
        canvas.drawRect(startZone, drawPaint);
        drawPaint.setColor(Color.GREEN);
        canvas.drawRect(endZone, drawPaint);
        
        final int count = mParticleSystem.getParticleCount();
        
        for (int i = 0; i < count; i++)
        {
            canvas.drawBitmap(mBitmap, x - r, y - r, null);
        }
        
        mMaze.drawWalls(canvas);
        
        // Dump any debug draw information
        // DebugDraw.Dump(canvas);
        
        // and make sure to redraw asap
        invalidate();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy)
    {
    	// Do something?
    }
}