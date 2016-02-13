package Localization;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightPoller extends Thread
{
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private int count = 0;
	private double[] thetaOfLines = new double[4];
	private int index = 0;
	private float colorValue;
	private boolean blackLineDetected;
	private boolean done = false;

	public LightPoller(Odometer odo, SampleProvider colorSensor, float[] colorData)
	{
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}

	public void run()
	{
		while(Lab4.leftMotor.isMoving())
		{
			getColorValue();
			if (colorValue < 0.3 && count < 7) //buffer
			{
				count++;
				blackLineDetected = false;
			}

			else if (colorValue < 0.3) //passed buffer, black line detected
			{
				Sound.beep();
				thetaOfLines[index] = odo.getTheta();
				System.out.println(odo.getTheta());
				blackLineDetected  = true;
			}
			//if above 0.08, consider as not black
			if (colorValue > 0.3) 
			{
				if(blackLineDetected)
				{
					index++;
				}
				count = 0;
				blackLineDetected = false;
			} 
			try 
			{ 
				Thread.sleep(2); 
			} 
			catch(Exception e)
			{

			}	
		}
	}
	public float getColorValue()
	{
		colorSensor.fetchSample(colorData, 0);
		colorValue = colorData[0];
		return colorValue;
	}
	public double[] getThetaOfLines()
	{
		return thetaOfLines;
	}
	
	public int getIndex()
	{
		return index;
	}
}
