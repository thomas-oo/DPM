package Localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private int forwardSpeed = 200;
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private double[] thetaOfLines = new double[4];

	private boolean fourValues = false;
	private int lineCount;
	private double nowTheta;

	private double thetaY;
	private double thetaX;
	private double posX;
	private double posY;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double lightDist = 16;
	public static double rWheel = 2.15; //measure
	public static double dBase = 16.2;
	public static int bandCenter = 20;
	public static int bandWidth = 3;
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
		nav = new Navigation(leftMotor, rightMotor, odo,bandCenter,bandWidth,motorLow,motorHigh);
	}

	public void doLocalization() throws InterruptedException 
	{
		LightPoller lightPoller = new LightPoller(odo, colorSensor, colorData);
		rotate(2*Math.PI);
		lightPoller.start();
		lightPoller.join();
		thetaOfLines = lightPoller.getThetaOfLines();

		lineCount = lightPoller.getIndex();

		while(!fourValues ) //once it gets out of this loop, the robot should have detected 4 lines and have stored all the values of theta
		{
			if(lineCount<4)
			{
				//have the robot move along the 45degree line
				turnTo(0.25*Math.PI);
				setSpeeds(50,50);
				Lab4.leftMotor.rotate(45,true);
				Lab4.rightMotor.rotate(45);
				turnTo(0); //place the heading back to normal

				lightPoller = new LightPoller(odo, colorSensor, colorData);
				rotate(2*Math.PI);
				lightPoller.start();
				lightPoller.join();
				thetaOfLines = lightPoller.getThetaOfLines();

				lineCount = lightPoller.getIndex();

			}
			else if(lineCount==4)
			{
				fourValues=true;
				break;
			}
			else
			{
				System.out.println("ERROR");
			}
		}

		for(double i : thetaOfLines)
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

		
		System.out.println("posX: " + posX + " posY: " + posY);
		//call navigation and make the robot move to point 0.0
		nav.start();
		nav.travelTo(-posX,-posY);
		
		nav.join();
		turnTo(0);
		
		System.out.println("DONE");
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
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
