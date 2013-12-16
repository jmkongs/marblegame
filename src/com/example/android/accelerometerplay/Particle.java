package com.example.android.accelerometerplay;

import android.graphics.RectF;
import android.util.Log;

/*
 * Each of our particle holds its previous and current position, its
 * acceleration. for added realism each particle has its own friction
 * coefficient.
 */
public class Particle
{
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
    private int mBallDiameter;
    
    // friction of the virtual table and air
    private static final float sFriction = 0.1f;
    
    // diameter of the balls in meters

    Particle( float metersPerPixelX, float metersPerPixelY, float originX, float originY, int ballDiameter )
    {
    	mBallDiameter = ballDiameter;
    	
        // make each particle a bit different by randomizing its
        // coefficient of friction
        final float r = ((float) Math.random() - 0.5f) * 0.2f;
        mOneMinusFriction = 1.0f - sFriction + r;
        mMetersPerPixelX = metersPerPixelX;
        mMetersPerPixelY = metersPerPixelY;
        mOriginX = originX;
        mOriginY = originY;
    }
    
    public float getDiameter()
    {
    	return mBallDiameter;
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

    public void setPos( float x, float y )
    {
    	mPosX = mLastPosX = x;
    	mPosY = mLastPosY = y;
    }
    
    public float getLastPosX()
    {
    	return mLastPosX;
    }
    
    public float getLastPosY()
    {
    	return mLastPosY;
    }
    
    public void computePhysics(float sx, float sy, float dT, float dTC) {
        // Force of gravity applied to our virtual object
        float ax = -sx;
        float ay = sy;        
        float friction_x, friction_y;

        final float dTdT = dT * dT;
        float deltaX = mAccelX * dTdT * mMetersPerPixelX;
        float deltaY = mAccelY * dTdT * mMetersPerPixelY;
     
        friction_x = mOneMinusFriction * dTC * (mPosX - mPosXset);
        friction_y = mOneMinusFriction * dTC * (mPosY - mPosYset);
        friction_x = 0;
        friction_y = 0;
        mLastPosX = mPosX;
        mLastPosY = mPosY;
        
        final int max = 16;
        final int min = -16;
        
        if( deltaX > max )
        {
        	deltaX = max;
        }
        else if( deltaX < min )
        {
        	deltaX = min;
        }
        
        if( deltaY > max )
        {
        	deltaY = max;
        }
        else if( deltaY < min )
        {
        	deltaY = min;
        }
        
        mPosX += deltaX + friction_x;
        mPosY += deltaY + friction_y;
        mAccelX = ax;
        mAccelY = ay;
    }

    public void resolveCollisionWithBounds( RectF bounds )
    {
    	float r = this.mBallDiameter / 2.0f;
    	RectF modBounds = new RectF();
    	modBounds.top = bounds.top + r;
    	modBounds.left = bounds.left + r;
    	modBounds.bottom = bounds.bottom - r;
    	modBounds.right = bounds.right - r;
    	
        if ( mPosX > modBounds.right )
        {
            mPosX = modBounds.right;
        }
        else if ( mPosX < bounds.left )
        {
            mPosX = modBounds.left;
        }
        
        if ( mPosY > modBounds.bottom )
        {
            mPosY = modBounds.bottom;
        }
        else if ( mPosY < bounds.top )
        {
            mPosY = modBounds.top;
        }
    }

	/**
	 * @return the mPosX
	 */
	public float getPosX()
	{
		return mPosX;
	}

	/**
	 * @param mPosX the mPosX to set
	 */
	public void setPosX(float x)
	{
    	mPosX = mLastPosX = x;
	}

	/**
	 * @return the mPosY
	 */
	public float getPosY()
	{
		return mPosY;
	}

	/**
	 * @param mPosY the mPosY to set
	 */
	public void setPosY(float y)
	{
    	mPosY = mLastPosY = y;
	}
}
