package Localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private int forwardSpeed = 200;
	
	private Odometer odo;
	
	//fields for color sensor
	private SampleProvider colorSensor;
	private float[] colorData;
	
	private double[] thetaOfLines = new double[4]; //stores theta values when we detect a line

	private boolean countedFourLines = false; //flag
	
	private int lineCount;
	private double nowTheta;

	private double thetaY;
	private double thetaX;
	private double posX;
	private double posY;
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private double lightDist = 16; //distance between center of wheelbase and light sensor
	public static double rWheel = Lab4.rWheel;
	public static double dBase = Lab4.dBase;
	public static int motorLow = 100;
	public static int motorHigh = 200;
	private static Navigation nav;

	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) 
	{
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		leftMotor = Lab4.leftMotor;
		rightMotor = Lab4.rightMotor;
		nav = new Navigation(leftMotor, rightMotor, odo);
	}

	public void doLocalization() throws InterruptedException 
	{
		//intializes a lightPoller thread, which runs in parallel while the robot rotates 360 degrees
		//once the lightPoller thread is done, check how many lines it has counted (we require 4)
		LightPoller lightPoller = new LightPoller(odo, colorSensor, colorData);
		
		rotate(2*Math.PI);

		lightPoller.start();
		
		lightPoller.join();
		
		thetaOfLines = lightPoller.getThetaOfLines();
		lineCount = lightPoller.getIndex();
		
		
		while(!countedFourLines ) //once it gets out of this loop, the robot should have detected 4 lines and have stored all the values of theta
		{
			if(lineCount<4) //not desired, move to a new position along the 45 degree line to try again and detect lines
			{
				//have the robot move along the 45degree line
				turnTo(0.25*Math.PI);
				setSpeeds(50,50);
				Lab4.leftMotor.rotate(90,true);
				Lab4.rightMotor.rotate(90);
				turnTo(0); //places the heading back to normal

				//create a new lightPoller thread (old one has died and cannot be restarted) and rotate 360 degrees again
				//once the lightPoller thread is done, check how many lines it has counted (we require 4)
				lightPoller = new LightPoller(odo, colorSensor, colorData);
				rotate(2*Math.PI);
				
				lightPoller.start();
				
				lightPoller.join();
				
				thetaOfLines = lightPoller.getThetaOfLines();
				lineCount = lightPoller.getIndex();

			}
			else if(lineCount==4) //if we have detected 4 lines (desired), get out of this loop
			{
				countedFourLines = true;
				break;
			}
			else
			{
				System.out.println("ERROR");
			}
		}

		for(double i : thetaOfLines) //printer to debug
		{
			System.out.println(i);
		}
		
		Lab4.rightMotor.stop();
		Lab4.leftMotor.stop();
		
		//calculating the x position
		thetaY = thetaOfLines[3]-thetaOfLines[1]; //thetaY2 - thetaY1
		posX = -(lightDist)*Math.cos((thetaY)/2);

		//calculating the y position
		thetaX = thetaOfLines[2]-thetaOfLines[0]; //thetaX2 - thetaX1
		posY = -(lightDist)*Math.cos((thetaX)/2);
		
		System.out.println("posX: " + posX + " posY: " + posY); //printer to debug
		
		//call navigation and make the robot move to point 0.0
		nav.start();
		nav.travelTo(-posX,-posY);
		
		nav.join();
		
		turnTo(0);
		
		System.out.println("DONE");
	}
	private void rotate(double turnTheta) //rotates turnTheta cw
	{
		setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.

		Lab4.leftMotor.rotate(convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta), true);
		Lab4.rightMotor.rotate(-convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta), true);

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
	private void turnTo(double destTheta) //uses destTheta and nowTheta to calculate AND TURN required minimal angle.
	{
		nowTheta = odo.getTheta();

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
		Lab4.leftMotor.rotate(-convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta), true);
		Lab4.rightMotor.rotate(convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta));
	}
}
