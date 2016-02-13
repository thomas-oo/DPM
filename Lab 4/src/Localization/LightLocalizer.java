package Localization;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private int forwardSpeed = 200;
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private int count = 0;
	private double[] thetaOfLines = new double[4];
	private int index = 0;
	
	private float colorValue;
	private boolean blackLineDetected;
	private boolean done = false;
	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	public void doLocalization() 
	{
		rotate(360);
		while (!done) 
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
		}
		
		for(double i : thetaOfLines)
		{
			System.out.println(i);
		}
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
	}
	
	public float getColorValue()
	{
		colorSensor.fetchSample(colorData, 0);
		colorValue = colorData[0];
		return colorValue;
	}
	private void rotate(double turnTheta) //rotates turnTheta cw
	{
		setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.
		while( index < 3 || (2*Math.PI - odo.getTheta())> 0.0174533 )
		{
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
		}
		done = true;

	}
	private static void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		Lab4.leftMotor.setSpeed(leftSpeed);
		Lab4.rightMotor.setSpeed(rightSpeed);
	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width);
	}
}
