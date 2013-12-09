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
import android.os.Bundle;
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
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

import com.example.android.accelerometerplay.AccelerometerPlayActivity.SimulationView.Particle;
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

    private SimulationView mSimulationView;
    private SensorManager mSensorManager;
    private PowerManager mPowerManager;
    private WindowManager mWindowManager;
    private Display mDisplay;
    private WakeLock mWakeLock;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

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
    class SimulationView extends View implements SensorEventListener {
        // diameter of the balls in meters
        private static final float sBallDiameter = 0.002f;
        private static final float sBallDiameter2 = sBallDiameter * sBallDiameter;

        // friction of the virtual table and air
        private static final float sFriction = 0.1f;

        private Sensor mAccelerometer;
        private long mLastT;
        private float mLastDeltaT;

        private float mXDpi;
        private float mYDpi;
        private float mMetersToPixelsX;
        private float mMetersToPixelsY;
        private Bitmap mBitmap;
        private Bitmap mWood;
        private float mXOrigin;
        private float mYOrigin;
        private float mSensorX;
        private float mSensorY;
        private long mSensorTimeStamp;
        private long mCpuTimeStamp;
        private float mHorizontalBound;
        private float mVerticalBound;
        private MyMaze mMaze;
        private ParticleSystem mParticleSystem;

        /*
         * Each of our particle holds its previous and current position, its
         * acceleration. for added realism each particle has its own friction
         * coefficient.
         */
        public class Particle {
        	private float mPosXset;
        	private float mPosYset;
            private float mPosX;
            private float mPosY;
            private float mAccelX;
            private float mAccelY;
            private float mLastPosX;
            private float mLastPosY;
            private float mOneMinusFriction;
            private float mMetersPerPixelX;
            private float mMetersPerPixelY;
            private float mOriginX;
            private float mOriginY;

            Particle( float metersPerPixelX, float metersPerPixelY, float originX, float originY ) {
                // make each particle a bit different by randomizing its
                // coefficient of friction
                final float r = ((float) Math.random() - 0.5f) * 0.2f;
                mOneMinusFriction = 1.0f - sFriction + r;
                mMetersPerPixelX = metersPerPixelX;
                mMetersPerPixelY = metersPerPixelY;
                mOriginX = originX;
                mOriginY = originY;
            }
            
            public void setOrigin( float x, float y )
            {
            	mOriginX = x;
            	mOriginY = y;
            }
            
            public void clearAccel()
            {
            	mAccelX = 0;
            	mAccelY = 0;
            }

            public void setPosPixel( float x, float y )
            {
            	mPosXset = mPosX = (x - mXOrigin ) / mMetersPerPixelX;
            	mPosYset = mPosY = ( mYOrigin - y ) / mMetersPerPixelY;
            }
            
            public float getPosXPixel ()
            {
            	return mOriginX + ( mPosX * mMetersPerPixelX );
            }
            
            public float getPosYPixel()
            {
            	return mOriginY - ( mPosY * mMetersPerPixelY );
            }
            
            public float getLastPosXPixel ()
            {
            	return mOriginX + ( mLastPosX * mMetersPerPixelX );
            }
            
            public float getLastPosYPixel()
            {
            	return mOriginY - ( mLastPosY * mMetersPerPixelY );
            }
            
            public void computePhysics(float sx, float sy, float dT, float dTC) {
                // Force of gravity applied to our virtual object
                final float m = 500.0f; // mass of our virtual object
                final float gx = -sx * m;
                final float gy = -sy * m;

                /*
                 * ·F = mA <=> A = ·F / m We could simplify the code by
                 * completely eliminating "m" (the mass) from all the equations,
                 * but it would hide the concepts from this sample code.
                 */
                final float invm = 1.0f / m;
                float ax = gx * invm;
                float ay = gy * invm;


                // Limit acceleration
                if( ax > 0.3f )
                {
                	ax = 0.3f;
                }
                else if ( ax < -0.3f )
                {
                	ax = -0.3f;
                }
                
                if( ay > 0.3f )
                {
                	ay = 0.3f;
                }
                else if( ay < -0.3f )
                {
                	ay = -0.3f;
                }                

                /*
                 * Time-corrected Verlet integration The position Verlet
                 * integrator is defined as x(t+Æt) = x(t) + x(t) - x(t-Æt) +
                 * a(t)Ætö2 However, the above equation doesn't handle variable
                 * Æt very well, a time-corrected version is needed: x(t+Æt) =
                 * x(t) + (x(t) - x(t-Æt)) * (Æt/Æt_prev) + a(t)Ætö2 We also add
                 * a simple friction term (f) to the equation: x(t+Æt) = x(t) +
                 * (1-f) * (x(t) - x(t-Æt)) * (Æt/Æt_prev) + a(t)Ætö2
                 */
                final float dTdT = dT * dT;
                float deltaX = mAccelX * dTdT;
                float deltaY = mAccelY * dTdT;
                if( deltaX > 16 )
                {
                	deltaX = 16f;
                }
                else if( deltaX < -16 )
                {
                	deltaX = -16f;
                }
                
                if( deltaY > 16 )
                {
                	deltaY = 16f;
                }
                else if( deltaY < -16 )
                {
                	deltaY = -16f;
                }
                
                final float x = mPosX + mOneMinusFriction * dTC * (mPosX - mPosXset) + deltaX;
                final float y = mPosY + mOneMinusFriction * dTC * (mPosY - mPosYset) + deltaY;
                mPosXset = mLastPosX = mPosX;
                mPosYset = mLastPosY = mPosY;
                mPosX = x;
                mPosY = y;
                
                mAccelX = ax;
                mAccelY = ay;
            }

            /*
             * Resolving constraints and collisions with the Verlet integrator
             * can be very simple, we simply need to move a colliding or
             * constrained particle in such way that the constraint is
             * satisfied.
             */
            public void resolveCollisionWithBounds() {
                final float xmax = mHorizontalBound;
                final float ymax = mVerticalBound;
                final float x = mPosX;
                final float y = mPosY;
                if (x > xmax) {
                    mPosX = xmax;
                } else if (x < -xmax) {
                    mPosX = -xmax;
                }
                if (y > ymax) {
                    mPosY = ymax;
                } else if (y < -ymax) {
                    mPosY = -ymax;
                }
            }
        }
        
        /*
         * A particle system is just a collection of particles
         */
        class ParticleSystem {
            static final int NUM_PARTICLES = 1;
            private Particle mBalls[] = new Particle[NUM_PARTICLES];

            ParticleSystem(float metersPerPixelX, float metersPerPixelY, float originX, float originY) {
                /*
                 * Initially our particles have no speed or acceleration
                 */
                for (int i = 0; i < mBalls.length; i++) {
                    mBalls[i] = new Particle(metersPerPixelX, metersPerPixelY, originX, originY);
                }
            }

            
            public void setOrigin( float x, float y )
            {
                for (int i = 0; i < mBalls.length; i++) {
                    mBalls[i].setOrigin( x, y );
                }
            }
            
            /*
             * Update the position of each particle in the system using the
             * Verlet integrator.
             */
            private void updatePositions(float sx, float sy, long timestamp) {
                final long t = timestamp;
                if (mLastT != 0) {
                    final float dT = (float) (t - mLastT) * (1.0f / 1000000000.0f);
                    if (mLastDeltaT != 0) {
                        final float dTC = dT / mLastDeltaT;
                        final int count = mBalls.length;
                        for (int i = 0; i < count; i++) {
                            Particle ball = mBalls[i];
                            ball.computePhysics(sx, sy, dT, dTC);
                        }
                    }
                    mLastDeltaT = dT;
                }
                mLastT = t;
            }

            /*
             * Performs one iteration of the simulation. First updating the
             * position of all the particles and resolving the constraints and
             * collisions.
             */
            public void update(float sx, float sy, long now) {
                // update the system's positions
                updatePositions(sx, sy, now);

                // We do no more than a limited number of iterations
                final int NUM_MAX_ITERATIONS = 10;

                /*
                 * Resolve collisions, each particle is tested against every
                 * other particle for collision. If a collision is detected the
                 * particle is moved away using a virtual spring of infinite
                 * stiffness.
                 */
                boolean more = true;
                final int count = mBalls.length;
                for (int k = 0; k < NUM_MAX_ITERATIONS && more; k++) {
                    more = false;
                    for (int i = 0; i < count; i++) {
                        Particle curr = mBalls[i];
                        for (int j = i + 1; j < count; j++) {
                            Particle ball = mBalls[j];
                            float dx = ball.mPosX - curr.mPosX;
                            float dy = ball.mPosY - curr.mPosY;
                            float dd = dx * dx + dy * dy;
                            // Check for collisions
                            if (dd <= sBallDiameter2) {
                                /*
                                 * add a little bit of entropy, after nothing is
                                 * perfect in the universe.
                                 */
                                dx += ((float) Math.random() - 0.5f) * 0.0001f;
                                dy += ((float) Math.random() - 0.5f) * 0.0001f;
                                dd = dx * dx + dy * dy;
                                // simulate the spring
                                final float d = (float) Math.sqrt(dd);
                                final float c = (0.5f * (sBallDiameter - d)) / d;
                                curr.mPosX -= dx * c;
                                curr.mPosY -= dy * c;
                                ball.mPosX += dx * c;
                                ball.mPosY += dy * c;
                                more = true;
                            }
                        }
                        /*
                         * Finally make sure the particle doesn't intersects
                         * with the walls.
                         */
                        curr.resolveCollisionWithBounds();
                    }
                }
            }

            public int getParticleCount() {
                return mBalls.length;
            }

            public float getPosX(int i) {
                return mBalls[i].mPosX;
            }

            public float getPosY(int i) {
                return mBalls[i].mPosY;
            }
            
            public void checkForWallCollision(MyMaze maze, Canvas canvas)
            {
            	float x, y, r;
                for (int i = 0; i < mBalls.length; i++)
                {
                    Particle ball = mBalls[i];
                    r = sBallDiameter * mMetersToPixelsX / 2f;
                    x = mXOrigin + ( ball.mLastPosX * mMetersToPixelsX ) + r;
                    y = mYOrigin - ( ball.mLastPosY * mMetersToPixelsY ) + r;
                    maze.checkWalls( ball,  r, canvas);
                    Paint paint = new Paint();
                    //paint.setColor( Color.RED );
                    //canvas.drawRect( new RectF(bounds[0], bounds[1], bounds[2], bounds[3]), paint);
                    paint.setColor( Color.BLUE );
   
                    canvas.drawCircle( x, y, r, paint );
                }
            }
        }
        
        
        
        public void startSimulation() {
            /*
             * It is not necessary to get accelerometer events at a very high
             * rate, by using a slower rate (SENSOR_DELAY_UI), we get an
             * automatic low-pass filter, which "extracts" the gravity component
             * of the acceleration. As an added benefit, we use less power and
             * CPU resources.
             */
            mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
        }

        public void stopSimulation() {
            mSensorManager.unregisterListener(this);
        }

        public SimulationView(Context context) {
            super(context);
            mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

            DisplayMetrics metrics = new DisplayMetrics();
            getWindowManager().getDefaultDisplay().getMetrics(metrics);
            mXDpi = metrics.xdpi;
            mYDpi = metrics.ydpi;
            mMetersToPixelsX = mXDpi / 0.0254f;
            mMetersToPixelsY = mYDpi / 0.0254f;
            mParticleSystem = new ParticleSystem(mMetersToPixelsX, mMetersToPixelsY, 0, 0);
            // rescale the ball so it's about 0.5 cm on screen
            Bitmap ball = BitmapFactory.decodeResource(getResources(), R.drawable.ball);
            final int dstWidth = (int) (sBallDiameter * mMetersToPixelsX + 0.5f);
            final int dstHeight = (int) (sBallDiameter * mMetersToPixelsY + 0.5f);
            mBitmap = Bitmap.createScaledBitmap(ball, dstWidth, dstHeight, true);
            
            Options opts = new Options();
            opts.inDither = true;
            opts.inPreferredConfig = Bitmap.Config.RGB_565;
            mWood = BitmapFactory.decodeResource(getResources(), R.drawable.wood, opts);
            final int woodDstWidth = (int) (metrics.widthPixels);
            final int woodDstHeight = (int) (metrics.heightPixels);
            mWood = Bitmap.createScaledBitmap(mWood, woodDstWidth, woodDstHeight, true);
            mMaze = new MyMaze( 10, 15, metrics );
            //mMaze.solve();
        }

        @Override
        protected void onSizeChanged(int w, int h, int oldw, int oldh) {
            // compute the origin of the screen relative to the origin of
            // the bitmap
            mXOrigin = (w - mBitmap.getWidth()) * 0.5f;
            mYOrigin = (h - mBitmap.getHeight()) * 0.5f;
            mParticleSystem.setOrigin(mXOrigin, mYOrigin);
            mHorizontalBound = ((w / mMetersToPixelsX - sBallDiameter) * 0.5f);
            mVerticalBound = ((h / mMetersToPixelsY - sBallDiameter) * 0.5f);
        }

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
                return;
            /*
             * record the accelerometer data, the event's timestamp as well as
             * the current time. The latter is needed so we can calculate the
             * "present" time during rendering. In this application, we need to
             * take into account how the screen is rotated with respect to the
             * sensors (which always return data in a coordinate space aligned
             * to with the screen in its native orientation).
             */

            switch (mDisplay.getRotation()) {
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

        @Override
        protected void onDraw(Canvas canvas) {

        	 /*
             * draw the background
             */

            canvas.drawBitmap(mWood, 0, 0, null);

            /*
             * compute the new position of our object, based on accelerometer
             * data and present time.
             */

            final ParticleSystem particleSystem = mParticleSystem;
            final long now = mSensorTimeStamp + (System.nanoTime() - mCpuTimeStamp);
            final float sx = mSensorX / 10.0f;
            final float sy = mSensorY / 10.0f;

            particleSystem.update(sx, sy, now);

            particleSystem.checkForWallCollision(mMaze, canvas);
            final float xc = mXOrigin;
            final float yc = mYOrigin;
            final float xs = mMetersToPixelsX;
            final float ys = mMetersToPixelsY;
            final Bitmap bitmap = mBitmap;
            final int count = particleSystem.getParticleCount();
            for (int i = 0; i < count; i++)
            {
                /*
                 * We transform the canvas so that the coordinate system matches
                 * the sensors coordinate system with the origin in the center
                 * of the screen and the unit is the meter.
                 */

                final float x = xc + particleSystem.getPosX(i) * xs;
                final float y = yc - particleSystem.getPosY(i) * ys;
                canvas.drawBitmap(bitmap, x, y, null);
            }
            
            mMaze.drawWalls(canvas);
            
            DebugDraw.Dump(canvas);
            // and make sure to redraw asap
            invalidate();
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }
    }
    
    static public class DebugDraw
    {
    	private static ArrayList<Integer> rectFColors = new ArrayList<Integer>();
    	private static ArrayList<RectF> rectFs = new ArrayList<RectF>();
    	
    	static public void Dump( Canvas canvas )
    	{
    		synchronized( rectFs )
    		{
    			Paint paint = new Paint();
    			for( int i = 0; i < rectFs.size(); i++ )
    			{
    				paint.setColor(rectFColors.get(i));
    				canvas.drawRect( rectFs.get(i), paint );
    			}
    			rectFs.clear();
    			rectFColors.clear();
    		}
    	}
    	
    	public static void AddRectF( RectF rect, int color )
    	{
    		synchronized( rectFs )
    		{
    			rectFColors.add( color );
    			rectFs.add(rect);
    		}
    	}
    }

    class MyMaze
    {
      private float pixelCellSizeX, pixelCellSizeY;
      private int dimensionX, dimensionY; // dimension of maze
      private int gridDimensionX, gridDimensionY; // dimension of output grid
      private char[][] grid; // output grid
      private Cell[][] cells; // 2d array of Cells
      private Random random = new Random(); // The random object
      private ArrayList<RectF> walls = new ArrayList<RectF>();
      
      
      // initialize with x and y the same
      public MyMaze(int aDimension, DisplayMetrics metrics ) {
          // Initialize
          this(aDimension, aDimension, metrics);
      }
      // constructor
      public MyMaze(int xDimension, int yDimension, DisplayMetrics metrics) {
    	  pixelCellSizeX = (float)metrics.widthPixels  / (float)( xDimension * 4 + 1);
    	  pixelCellSizeY = (float)metrics.heightPixels / (float)( yDimension * 4 + 1);
          dimensionX = xDimension;
          dimensionY = yDimension;
          gridDimensionX = xDimension * 4 + 1;
          gridDimensionY = yDimension * 4 + 1;
          grid = new char[gridDimensionX][gridDimensionY];
          init();
          generateMaze();
          updateGrid();
      }

      private void init() {
          // create cells
          cells = new Cell[dimensionX][dimensionY];
          for (int x = 0; x < dimensionX; x++) {
              for (int y = 0; y < dimensionY; y++) {
                  cells[x][y] = new Cell(x, y, false); // create cell (see Cell constructor)
              }
          }
      }

      // inner class to represent a cell
      private class Cell {
        int x, y; // coordinates
        // cells this cell is connected to
        ArrayList<Cell> neighbors = new ArrayList<Cell>();
        // solver: if already used
        boolean visited = false;
        // solver: the Cell before this one in the path
        Cell parent = null;
        // solver: if used in last attempt to solve path
        boolean inPath = false;
        // solver: distance traveled this far
        double travelled;
        // solver: projected distance to end
        double projectedDist;
        // impassable cell
        boolean wall = true;
        // if true, has yet to be used in generation
        boolean open = true;
        // construct Cell at x, y
        Cell(int x, int y) {
            this(x, y, true);
        }
        // construct Cell at x, y and with whether it isWall
        Cell(int x, int y, boolean isWall) {
            this.x = x;
            this.y = y;
            this.wall = isWall;
        }
        // add a neighbor to this cell, and this cell as a neighbor to the other
        void addNeighbor(Cell other) {
            if (!this.neighbors.contains(other)) { // avoid duplicates
                this.neighbors.add(other);
            }
            if (!other.neighbors.contains(this)) { // avoid duplicates
                other.neighbors.add(this);
            }
        }
        // used in updateGrid()
        boolean isCellBelowNeighbor() {
            return this.neighbors.contains(new Cell(this.x, this.y + 1));
        }
        // used in updateGrid()
        boolean isCellRightNeighbor() {
            return this.neighbors.contains(new Cell(this.x + 1, this.y));
        }
        // useful Cell representation
        @Override
        public String toString() {
            return String.format("Cell(%s, %s)", x, y);
        }
        // useful Cell equivalence
        @Override
        public boolean equals(Object other) {
            if (!(other instanceof Cell)) return false;
            Cell otherCell = (Cell) other;
            return (this.x == otherCell.x && this.y == otherCell.y);
        }
        // should be overridden with equals
        @Override
        public int hashCode() {
            // random hash code method designed to be usually unique
            return this.x + this.y * 256;
        }
      }
      // generate from upper left (In computing the y increases down often)
      private void generateMaze() {
          generateMaze(0, 0);
      }
      // generate the maze from coordinates x, y
      private void generateMaze(int x, int y) {
          generateMaze(getCell(x, y)); // generate from Cell
      }
      
      private void checkWalls( Particle ball, float r, Canvas canvas )
      {
    	  float x, y, d, lastX, lastY, newX, newY;
    	  x = ball.getPosXPixel();
    	  y = ball.getPosYPixel();
    	  lastX = ball.getLastPosXPixel();
    	  lastY = ball.getLastPosYPixel();
    	  d = 2 * r;
    	  
    	  
    	  RectF objLast = new RectF(lastX, lastY, lastX + d, lastY + d );
    	  for( int i = 0; i < walls.size(); i++ )
    	  {
        	  RectF obj = new RectF(x, y, x + d, y + d );
    		  RectF wall = walls.get(i); 		  
    		  
    		  if( obj.intersect(wall) )
    		  {
    			  // left or right
				  if( obj.centerX() < objLast.centerX() )
				  {
					  newX = x + obj.width();
				  }
				  else
				  {
					  newX = x - obj.width();
				  }
				  
				  // top or bottom
				  if( obj.centerY() < objLast.centerY() )
    			  {
					  newY = y + obj.height();
    			  }
				  else
				  {
    				newY = y - obj.height();
				  }
				  
				  if( obj.top == wall.top || obj.bottom == wall.bottom )
				  {
					  y = newY;
				  }
				  else
				  {
					  x = newX;
				  }
				  ball.setPosPixel(x, y);
				 
				  //DebugDraw.AddRectF(new RectF(x, y, x+d, y+d), Color.GREEN);
				  //DebugDraw.AddRectF(obj, Color.MAGENTA);
				  
				  RectF obj2 = new RectF( x, y, x+d, y+d);
				  if( obj2.intersect(wall) )
				  {
					  DebugDraw.AddRectF(obj2, Color.BLUE );
				  }
    		  }
    	  }
      }
      
      private void generateMaze(Cell startAt) {
          // don't generate from cell not there
          if (startAt == null) return;
          startAt.open = false; // indicate cell closed for generation
          ArrayList<Cell> cells = new ArrayList<Cell>();
          cells.add(startAt);

          while (!cells.isEmpty()) {
              Cell cell;
              // this is to reduce but not completely eliminate the number
              //   of long twisting halls with short easy to detect branches
              //   which results in easy mazes
              if (random.nextInt(10)==0)
                  cell = cells.remove(random.nextInt(cells.size()));
              else cell = cells.remove(cells.size() - 1);
              // for collection
              ArrayList<Cell> neighbors = new ArrayList<Cell>();
              // cells that could potentially be neighbors
              Cell[] potentialNeighbors = new Cell[]{
                  getCell(cell.x + 1, cell.y),
                  getCell(cell.x, cell.y + 1),
                  getCell(cell.x - 1, cell.y),
                  getCell(cell.x, cell.y - 1)
              };
              for (Cell other : potentialNeighbors) {
                  // skip if outside, is a wall or is not opened
                  if (other==null || other.wall || !other.open) continue;
                  neighbors.add(other);
              }
              if (neighbors.isEmpty()) continue;
              // get random cell
              Cell selected = neighbors.get(random.nextInt(neighbors.size()));
              // add as neighbor
              selected.open = false; // indicate cell closed for generation
              cell.addNeighbor(selected);
              cells.add(cell);
              cells.add(selected);
          }
      }
      // used to get a Cell at x, y; returns null out of bounds
      public Cell getCell(int x, int y) {
          try {
              return cells[x][y];
          } catch (ArrayIndexOutOfBoundsException e) { // catch out of bounds
              return null;
          }
      }

      public void solve() {
          // default solve top left to bottom right
          this.solve(0, 0, dimensionX - 1, dimensionY -1);
      }
      // solve the maze starting from the start state (A-star algorithm)
      public void solve(int startX, int startY, int endX, int endY) {
          // re initialize cells for path finding
          for (Cell[] cellrow : this.cells) {
              for (Cell cell : cellrow) {
                  cell.parent = null;
                  cell.visited = false;
                  cell.inPath = false;
                  cell.travelled = 0;
                  cell.projectedDist = -1;
              }
          }
          // cells still being considered
          ArrayList<Cell> openCells = new ArrayList<Cell>();
          // cell being considered
          Cell endCell = getCell(endX, endY);
          if (endCell == null) return; // quit if end out of bounds
          { // anonymous block to delete start, because not used later
              Cell start = getCell(startX, startY);
              if (start == null) return; // quit if start out of bounds
              start.projectedDist = getProjectedDistance(start, 0, endCell);
              start.visited = true;
              openCells.add(start);
          }
          boolean solving = true;
          while (solving) {
              if (openCells.isEmpty()) return; // quit, no path
              // sort openCells according to least projected distance
              Collections.sort(openCells, new Comparator<Cell>(){
                  @Override
                  public int compare(Cell cell1, Cell cell2) {
                      double diff = cell1.projectedDist - cell2.projectedDist;
                      if (diff > 0) return 1;
                      else if (diff < 0) return -1;
                      else return 0;
                  }
              });
              Cell current = openCells.remove(0); // pop cell least projectedDist
              if (current == endCell) break; // at end
              for (Cell neighbor : current.neighbors) {
                  double projDist = getProjectedDistance(neighbor,
                          current.travelled + 1, endCell);
                  if (!neighbor.visited || // not visited yet
                          projDist < neighbor.projectedDist) { // better path
                      neighbor.parent = current;
                      neighbor.visited = true;
                      neighbor.projectedDist = projDist;
                      neighbor.travelled = current.travelled + 1;
                      if (!openCells.contains(neighbor))
                          openCells.add(neighbor);
                  }
              }
          }
          // create path from end to beginning
          Cell backtracking = endCell;
          backtracking.inPath = true;
          while (backtracking.parent != null) {
              backtracking = backtracking.parent;
              backtracking.inPath = true;
          }
      }
      // get the projected distance
      // (A star algorithm consistent)
      public double getProjectedDistance(Cell current, double travelled, Cell end) {
          return travelled + Math.abs(current.x - end.x) + 
                  Math.abs(current.y - current.x);
      }
      
      // draw the maze
      public void updateGrid() {
          char backChar = ' ', wallChar = '+', cellChar = ' ', pathChar = '*';
          // fill background
          for (int x = 0; x < gridDimensionX; x ++) {
              for (int y = 0; y < gridDimensionY; y ++) {
                  grid[x][y] = backChar;
              }
          }
          // build walls
          for (int x = 0; x < gridDimensionX; x ++) {
              for (int y = 0; y < gridDimensionY; y ++) {
                  if (x % 4 == 0 || y % 4 == 0)
                      grid[x][y] = wallChar;
              }
          }
          
          // make meaningful representation
          for (int x = 0; x < dimensionX; x++)
          {
              for (int y = 0; y < dimensionY; y++)
              {
                  Cell current = getCell(x, y);
                  int gridX = x * 4 + 2, gridY = y * 4 + 2;
                  if (current.inPath) {
                      grid[gridX][gridY] = pathChar;
                      if (current.isCellBelowNeighbor())
                          if (getCell(x, y + 1).inPath)
                          {
                        	  grid[gridX - 1][gridY + 1] = backChar;
                        	  grid[gridX][gridY + 1] = pathChar;
                        	  grid[gridX + 1][gridY + 1] = backChar;
                        	  
                        	  grid[gridX - 1][gridY + 2] = backChar;
                        	  grid[gridX][gridY + 2] = pathChar;
                        	  grid[gridX + 1][gridY + 2] = backChar;
                        	  
                        	  grid[gridX - 1][gridY + 3] = backChar;
                        	  grid[gridX][gridY + 3] = pathChar;
                        	  grid[gridX + 1][gridY + 3] = backChar;                   
                          }
                          else
                          {
                        	  grid[gridX - 1][gridY + 1] = backChar;
                        	  grid[gridX][gridY + 1] = cellChar;
                        	  grid[gridX + 1][gridY + 1] = backChar;
                        	  
                        	  grid[gridX - 1][gridY + 2] = backChar;
                        	  grid[gridX][gridY + 2] = cellChar;
                        	  grid[gridX + 1][gridY + 2] = backChar;
                        	  
                        	  grid[gridX - 1][gridY + 3] = backChar;
                        	  grid[gridX][gridY + 3] = cellChar;
                        	  grid[gridX + 1][gridY + 3] = backChar;  
                          }
                      if (current.isCellRightNeighbor())
                          if (getCell(x + 1, y).inPath)
                          {
                        	  grid[gridX + 1][gridY - 1] = backChar;
                        	  grid[gridX + 1][gridY]     = pathChar;
                              grid[gridX + 1][gridY + 1] = backChar;
                        	  
                        	  grid[gridX + 2][gridY - 1] = backChar;
                        	  grid[gridX + 2][gridY]     = pathChar;
                              grid[gridX + 2][gridY + 1] = backChar;
                              
                        	  grid[gridX + 3][gridY - 1] = backChar;
                        	  grid[gridX + 3][gridY]     = pathChar;
                              grid[gridX + 3][gridY + 1] = backChar;
                          } else
                          {
                        	  grid[gridX + 1][gridY - 1] = backChar;
                        	  grid[gridX + 1][gridY]     = cellChar;
                              grid[gridX + 1][gridY + 1] = backChar;
                        	  
                        	  grid[gridX + 2][gridY - 1] = backChar;
                        	  grid[gridX + 2][gridY]     = cellChar;
                              grid[gridX + 2][gridY + 1] = backChar;
                              
                        	  grid[gridX + 3][gridY - 1] = backChar;
                        	  grid[gridX + 3][gridY]     = cellChar;
                              grid[gridX + 3][gridY + 1] = backChar;
                          }
                  }
                  else
                  {
                      grid[gridX][gridY] = cellChar;
                      if (current.isCellBelowNeighbor())
                      {
                          grid[gridX - 1][gridY + 1] = backChar;
                          grid[gridX][gridY + 1] = cellChar;
                          grid[gridX + 1][gridY + 1] = backChar;

                          grid[gridX - 1][gridY + 2] = backChar;
                          grid[gridX][gridY + 2] = cellChar;
                          grid[gridX + 1][gridY + 2] = backChar;

                          grid[gridX - 1][gridY + 3] = backChar;
                          grid[gridX][gridY + 3] = cellChar;
                          grid[gridX + 1][gridY + 3] = backChar;

                      }
                      if (current.isCellRightNeighbor())
                      {
                          grid[gridX + 1][gridY - 1] = backChar;
                          grid[gridX + 1][gridY] = cellChar;
                          grid[gridX + 1][gridY + 1] = backChar;
                          
                          grid[gridX + 2][gridY - 1] = backChar;
                          grid[gridX + 2][gridY] = cellChar;
                          grid[gridX + 2][gridY + 1] = backChar;
                          
                          grid[gridX + 3][gridY - 1] = backChar;
                          grid[gridX + 3][gridY] = cellChar;
                          grid[gridX + 3][gridY + 1] = backChar;
                      }
                  }
              }
          }
          
          // Make collidable walls
          boolean wallUsed[][][] = new boolean[gridDimensionX][gridDimensionY][2];
          
          for( int i= 0; i < gridDimensionY; i++ )
          {
        	  for( int j= 0; j < gridDimensionX; j++ )
        	  {
        		  wallUsed[j][i][0] = false;
        		  wallUsed[j][i][1] = false;
        	  }
          }
          
          walls.clear();
          // for each row..
          for( int i = 0; i < gridDimensionY; i++ )
          {
        	  for( int j = 0; j < gridDimensionX; j++ )
        	  {
        		  if( j < gridDimensionX - 1 && !wallUsed[j][i][0] && grid[j][i] == '+' && grid[j + 1][i] == '+' && !wallUsed[j+1][i][0] )
        		  {
        			  // horizontal wall
        			  wallUsed[j][i][0]   = true;
        			  wallUsed[j+1][i][0] = true;
        			  int k;
        			  for( k = j + 2; k < gridDimensionX && grid[k][i] == '+' && !wallUsed[k][i][0]; k++ )
        			  {
        				  wallUsed[k][i][0] = true;
        			  }
        			  
        		      RectF wall = new RectF();
        		      wall.left = ( (float)j /*+ 0.5f*/ ) * pixelCellSizeX;
        		      wall.top = (float)i * pixelCellSizeY;
        		      wall.bottom = wall.top + pixelCellSizeY;
        		      wall.right = ( (float)k/* - 0.5f */) * pixelCellSizeX;
        		      walls.add(wall);
        		  } 
        		  else if ( i < gridDimensionY - 1 && !wallUsed[j][i][1] && grid[j][i] == '+' && grid[j][i+1] == '+' && !wallUsed[j][i+1][1] )
        		  {
        			  // vertical wall
        			  wallUsed[j][i][1]   = true;
        			  wallUsed[j][i+1][1] = true;
        			  int k;
        			  for( k = i + 2; k < gridDimensionY && grid[j][k] == '+' && !wallUsed[j][k][1]; k++ )
        			  {
        				  wallUsed[j][k][1] = true;
        			  }
        			  
        		      RectF wall = new RectF();
        		      wall.left = (float)j * pixelCellSizeX;
        		      wall.top = ( (float)i /*+ 0.5f*/ ) * pixelCellSizeY;
        		      wall.bottom = ( (float)k/* - 0.5f*/ ) * pixelCellSizeY;
        		      wall.right = wall.left + pixelCellSizeX;
        		      walls.add(wall);
        		  }
        	  }
          }
      }
      
      public void drawWalls( Canvas canvas )
      {
          if( canvas != null )
          {
        	  Paint paint = new Paint();
        	  paint.setColor( Color.WHITE );
        	  for( int i = 0; i < walls.size(); i++ )
        	  {
        		  canvas.drawRect( walls.get(i), paint);
        	  }
          }
      }
      
      // forms a meaningful representation
      @Override
      public String toString() {
          updateGrid();
          String output = "";
          for (int y = 0; y < gridDimensionY; y++) {
              for (int x = 0; x < gridDimensionX; x++) {
                  output += grid[x][y];
              }
              output += "\n";
          }
          return output;
      }

    }
    
}
