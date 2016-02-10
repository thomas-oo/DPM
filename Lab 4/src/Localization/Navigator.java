package Localization;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigator extends Thread
{
	private int forwardSpeed = 150;

	//destination variables
	private double[] destDistance = new double[2];
	private double destTheta;

	//current coordinates
	private double[] nowDistance = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;

	//thresholds to allow control to within this window.
	private double thetaThreshold = 0.00873; //2 degrees in rads
	private double destThreshold = 0.4; //0.5 cm of deviation allowed in navigating to waypoint

	//state variables
	private boolean isNavigating;
	private boolean isCorrecting;

	//motors that need to be used
	private EV3LargeRegulatedMotor leftMotor, rightMotor, headMotor;

	//classes that navigator depends on
	private Odometer odometer;

	//buffers for accessing ultrasonic data
	public float[] usData;
	public double usDistance;
	public SampleProvider usSampleProvider;
	public SensorModes usSensor;

	//variables that are set in main, passed here
	private final int bandCenter, bandWidth;
	private final int motorLow, motorHigh;

	private double dBase; //give values
	private double rWheel;

	private static final Port usPort = LocalEV3.get().getPort("S2");

	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor headMotor, Odometer odometer, int bandCenter, int bandWidth, int motorLow, int motorHigh)
	{
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.headMotor = headMotor;

		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.motorLow = motorLow; 
		this.motorHigh = motorHigh;

		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usSampleProvider = usSensor.getMode("Distance");
		this.usData = new float[usSampleProvider.sampleSize()];
	}

	enum State {INIT,WALL,TURNING, TRAVELING};

	public void run()
	{

		try
		{
			Thread.sleep(30);
		}
		catch(InterruptedException e)
		{
			e.printStackTrace();
		} 	
	}



	private boolean checkEmergency() { //checking if it's too close to the wall
		if(usDistance <= 20)
			return true;
		else 
			return false;
	}
	private void updateTravel() //update the destination angle of the next "cycle" of travel
	{
		destTheta = getDestAngle();
	}
	private boolean facingDest(double destTheta) //checks if nowTheta is facing destTheta within a leeway of thetaThreshold 
	{
		if(nowTheta > (destTheta - thetaThreshold) && nowTheta < (destTheta + thetaThreshold))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	private boolean checkIfDone(double[] nowDistance) //checks if nowX, Y are within destThreshold of destDistnace[0], [1]
	{
		if(nowX > (destDistance[0]-destThreshold) && (nowX < (destDistance[0] + destThreshold)))
		{
			if(nowY > (destDistance[1]-destThreshold) && (nowY < (destDistance[1]+ destThreshold)))
			{
				return true;
			}
			return false;
		}
		return false;
	}
	private void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		isNavigating();
	}
	public void travelTo(double destX, double destY) //called from main, (never called inside navigator). Is basically a setter for destination and flags isNavigating to true
	{
		destDistance[0] = destX;
		destDistance[1] = destY;
		destTheta = getDestAngle();
		isNavigating = true;
	}
	private double getDestAngle() //uses destDistance[0],[1] to calculate destTheta. destTheta is the heading with respect to coordinate system. 
	//(where the min is 0 (on positive x-axis), and max is 2pi. positive theta is counterclockwise angle from the positive x-axis.
	{
		double errorX = destDistance[0] - nowX;
		double errorY = destDistance[1] - nowY;

		if(Math.abs(errorX) < destThreshold)
		{
			if(errorY > 0)
			{
				return 0.5 * Math.PI; //90
			}
			else
			{
				return 1.5 * Math.PI; //270
			}
		}
		else if(Math.abs(errorY) < destThreshold)
		{
			if(errorX > 0)
			{
				return 0.0; //0
			}
			else
			{
				return Math.PI; //180
			}
		}


		else if(errorX > 0) 
		{
			if(errorY > 0) //positive theta
			{
				return Math.atan(errorY/errorX);
			}
			else //converts quadrant 4 into a positive theta
			{
				return 2*Math.PI + Math.atan(errorY/errorX);
			}
		}
		else if(errorX < 0)
		{
			if(errorY > 0) //quad 2, positive theta
			{
				return (Math.atan(errorY/errorX) + Math.PI);
			}
			else if(errorY < 0) //quad 3, positive theta
			{
				return (Math.atan(errorY/errorX) + Math.PI);
			}
		}
		return 0; //IF SOMETHING GOES HORRIBLY WRONG, RETURN 0.
	}
	private void turnTo(double destTheta) //uses destTheta and nowTheta to calculate AND TURN required minimal angle.
	{
		setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.

		double turnTheta = destTheta - nowTheta; //dest and nowTheta both are from [0,2pi]
		//CALCULATES MINIMAL TURN and EXECUTES
		//ROTATES UNTIL TURN IS COMPLETE.
		if(turnTheta >= -Math.PI && turnTheta <= Math.PI)
		{
		}
		else if(turnTheta < -Math.PI && turnTheta > -2*Math.PI)
		{
			turnTheta = turnTheta + 2*Math.PI;
			System.out.println("a");
		}
		else if(turnTheta>Math.PI && turnTheta < 2*Math.PI)
		{
			turnTheta = turnTheta - 2*Math.PI;
			System.out.println("b");
		}
		else
		{
			System.out.println("turnTheta error: " + turnTheta);
		}
		leftMotor.rotate(-convertAngle(rWheel, dBase, turnTheta), true);
		rightMotor.rotate(convertAngle(rWheel, dBase, turnTheta));
	}
	public boolean isNavigating() //returns true if another thread has called travelTo or turnTo
	{
		return isNavigating;
	}
	public boolean isCorrecting() //returns true if navigator is currently turning
	{
		return isCorrecting;
	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width);
	}
	public void setNavigating(boolean isNavigating) //sets isNavigating if ever needed (unused here)
	{
		this.isNavigating = isNavigating;
	}
}
