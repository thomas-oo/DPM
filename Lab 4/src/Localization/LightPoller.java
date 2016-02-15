package Localization;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightPoller extends Thread
{
	private Odometer odo;
	
	//fields for color sensor
	private SampleProvider colorSensor;
	private float[] colorData;
	private int count = 0;
	private float colorValue; //value of the most recent color value read
	
	private double[] thetaOfLines = new double[4]; //stores theta values when we detect a line
	private int index = 0; //used as index for which to store a theta value, also doubles as a line counter!
	
	private boolean blackLineDetected; //flag

	public LightPoller(Odometer odo, SampleProvider colorSensor, float[] colorData)
	{
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}

	public void run()
	{
		//thread dies when the leftmotor stops moving
		
		while(Lab4.leftMotor.isMoving())
		{
			getColorValue();
			if (colorValue < 0.3 && count < 7) //buffer so a line isn't detected sporadically
			{
				count++;
				blackLineDetected = false;
			}

			else if (colorValue < 0.3) //passed buffer, black line detected
			{
				Sound.beep();
				
				thetaOfLines[index] = odo.getTheta(); //store the theta reading of the odometer into the array at the current index
				System.out.println(odo.getTheta());
				
				blackLineDetected  = true; //raise flag
			}
			if (colorValue > 0.3) //when black line isn't detected
			{
				if(blackLineDetected) //if the flag is raised (as in, we just detected a black line, but now we do not)
				{
					index++; //increase the index (we have PASSED the black line we just detected)
				}
				count = 0; //resets count
				
				blackLineDetected = false; //pulls flag down
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
	public void getColorValue() //get the most recent color reading and set it to colorValue
	{
		colorSensor.fetchSample(colorData, 0);
		colorValue = colorData[0];
	}
	public double[] getThetaOfLines() //a getter for the array thetaOfLines
	{
		return thetaOfLines;
	}
	
	public int getIndex() //a getter for the current index. (after 4th line is detected, index will be 4, thus doubles as a line counter)
	{
		return index;
	}
}
