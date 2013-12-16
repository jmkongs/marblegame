package com.example.android.accelerometerplay;

import java.util.ArrayList;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

/*
 * A particle system is just a collection of particles
 */
public class ParticleSystem
{
    private long mLastT = 0;
    private float mLastDeltaT = 0;
    
    private ArrayList<Particle> mBalls = new ArrayList<Particle>();

    ParticleSystem( float originX, float originY )
    {
    	setOrigin( originX, originY );   
    }
    
    
    public void addParticle( Particle p )
    {
    	mBalls.add(p);
    }
    
    public void setOrigin( float x, float y )
    {
        for (int i = 0; i < mBalls.size(); i++)
        {
            mBalls.get(i).setOrigin( x, y );
        }
    }
    
    /* Update the position of each particle in the system using the
     * Verlet integrator.
     */
    private void updatePositions(float sx, float sy, long timestamp)
    {
        final long t = timestamp;
        if ( mLastT != 0 )
        {
            final float dT = (float) (t - mLastT) * (1.0f / 1000000000.0f);
            if ( mLastDeltaT != 0 )
            {
                final float dTC = dT / mLastDeltaT;
                for (int i = 0; i < mBalls.size(); i++)
                {
                    Particle ball = mBalls.get(i);
                    ball.computePhysics(sx, sy, dT, dTC);
                }
            }
            mLastDeltaT = dT;
        }
        mLastT = t;
    }
    
    public Particle getParticle( int i )
    {
    	return mBalls.get(i);
    }

    /*
     * Performs one iteration of the simulation. First updating the
     * position of all the particles and resolving the constraints and
     * collisions.
     */
    public void update( float sx, float sy, long now )
    {
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
        final int count = mBalls.size();
        for (int k = 0; k < NUM_MAX_ITERATIONS && more; k++)
        {
            more = false;
            for (int i = 0; i < count; i++)
            {
                Particle curr = mBalls.get(i);
                for (int j = i + 1; j < count; j++)
                {
                    Particle ball = mBalls.get(j);
                    float effective_diameter = ( ( curr.getDiameter() + ball.getDiameter() ) / 2 );
                    float dx = ball.getPosX() - curr.getPosX();
                    float dy = ball.getPosY() - curr.getPosY();
                    float dd = dx * dx + dy * dy;
                    
                    // Check for collisions
                    if (dd <= effective_diameter )
                    {
                        /* add a little bit of entropy, after nothing is
                         * perfect in the universe.
                         */
                        dx += ((float) Math.random() - 0.5f) * 0.0001f;
                        dy += ((float) Math.random() - 0.5f) * 0.0001f;
                        dd = dx * dx + dy * dy;
                        
                        // simulate the spring
                        final float d = (float) Math.sqrt(dd);
                        final float c = (0.5f * (effective_diameter - d)) / d;
                        curr.setPosX(curr.getPosX() - dx * c);
                        curr.setPosY(curr.getPosY() - dy * c);
                        ball.setPosX(ball.getPosX() + dx * c);
                        ball.setPosY(ball.getPosY() + dy * c);
                        more = true;
                    }
                }
                
                /* Finally make sure the particle doesn't intersects
                 * with the walls.
                 */
                curr.resolveCollisionWithBounds(mBounds);
            }
        }
    }

    public int getParticleCount()
    {
        return mBalls.size();
    }

    public float getPosX(int i)
    {
        return mBalls.get(i).getPosX();
    }

    public float getPosY(int i)
    {
        return mBalls.get(i).getPosY();
    }
    
    private RectF mBounds;
    
    public void setBounds( RectF bounds )
    {
    	mBounds = bounds;
    }
    
    public void checkForWallCollision(Maze maze, Canvas canvas)
    {
    	float x, y, d;
        for (int i = 0; i < mBalls.size(); i++)
        {
            Particle ball = mBalls.get(i);
            maze.checkWalls( ball, canvas);
        }
    }
}
