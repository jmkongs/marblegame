package com.example.android.accelerometerplay;

import java.util.ArrayList;
import java.util.Queue;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

final public class DebugDraw
{
	private static ArrayList<Integer> rectFColors = new ArrayList<Integer>();
	private static ArrayList<RectF> rectFs = new ArrayList<RectF>();
	private static String text;
	
	static public void WriteText( String p_text )
	{
		text = p_text;
	}
	
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
		
		if( text != null && !text.isEmpty())
		{
			Paint paint = new Paint();
			paint.setTextSize(30);
		
			float width = paint.measureText(text);

			paint.setColor(Color.BLACK);
			canvas.drawRect(new RectF(0, 470, width + 20, 500 ), paint);
			paint.setColor(Color.WHITE);
			canvas.drawText(text, 20, 500, paint);
			text = null;
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