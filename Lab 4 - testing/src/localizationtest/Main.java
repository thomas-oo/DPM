package localizationtest;

import localizationtest.Odometer;

import java.io.IOException;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main 
{
	public static double rWheel = 2.1; //give values
	public static double dBase = 15.7;
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");
	public static int numSamples = 360;
	public static long samplePeriod = 17;
	private static int d = 35;
	public static double[] distData = new double[numSamples]; //please calculate how much space we need.
	public static double[] thetaData = new double[numSamples];
	public static boolean faceWall;
	
	private static int forwardSpeed = 250;
	private static double deltaTheta;
	private static double alpha;
	private static double beta;
	private static Odometer odo;
	private static double nowTheta;
	private static double fallingEdge;
	private static double risingEdge;

	public static void main(String[] args) throws IOException 
	{
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];

		odo = new Odometer(rWheel, dBase, samplePeriod);
		odo.start();

		USPoller usPoller = new USPoller(odo, usValue, usData,numSamples, samplePeriod);
		usPoller.start();

		rotate(2*Math.PI);
		
/*		System.out.println("THETA: ");
		for(int i = 0; i <thetaData.length; i++)
		{
			System.out.println(thetaData[i]);
		}
		System.out.println("DISTANCE: ");
		for(int i = 0; i <distData.length; i++)
		{
			System.out.println(distData[i]);
		}*/
		
		if(distData[0] <40)
		{
			faceWall = true;
		}
		else
		{
			faceWall = false;
		}
		
		for (int i=1;i<numSamples;i++)
		{
			if(distData[i-1]>d && distData[i]<d)//if the distance data point drops under 40 //falling edge
			{
				Sound.beep();
				double fallingDistance = distData[i-1]-40; //the distance of the point before the drop -40 (d)
				double thetaDifference = thetaData[i]-thetaData[i-1]; //(a)
				double distDifference = distData[i-1]-distData[i]; //(b)
				
				//c=(d*a)/b
				double thetaFall = (fallingDistance*thetaDifference)/distDifference; //(c)
				fallingEdge = thetaFall+thetaData[i-1]; //(c+40)
				
				
			}
			if (distData[i-1]<d && distData[i]>d) //rising edge (alpha)
			{
				Sound.buzz();
				double distDifference = distData[i]-distData[i-1]; //(b)
				double risingDistance = distDifference-(distData[i]-40); //the distance of the point before the rise to 40 (d)
				double thetaDifference = thetaData[i]-thetaData[i-1]; //(a)
				
				//c=(d*a)/b
				double thetaFall = (risingDistance*thetaDifference)/distDifference; //(c)
				risingEdge = thetaFall+thetaData[i-1]; //(c+40)
			}
		}
		
		beta = (risingEdge - fallingEdge)/4 + fallingEdge;
		alpha = risingEdge - (risingEdge - fallingEdge)/4;

		System.out.println("Alpha: " + (int)(alpha*57.296));
		System.out.println("Beta: " + (int)(beta*57.296));
		if (faceWall) //A<B
		{	
			deltaTheta = 0.25*Math.PI -((alpha+beta)/2);
			odo.setTheta(deltaTheta);
			System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (int)((deltaTheta)*57.296) + "turning to 0");
			turnTo(0);
		}
		else// not facing the wall, A>B
		{
			deltaTheta = 1.25*Math.PI -((alpha+beta)/2);
			odo.setTheta(deltaTheta);
			System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (int)((deltaTheta)*57.296) + "turning to 0");
			turnTo(0);
		}
		
	}

	private static void rotate(double turnTheta) //rotates turnTheta ccw (in same direction as odometer increases theta)
	{
		setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.

		leftMotor.rotate(-convertAngle(rWheel, dBase, turnTheta), true);
		rightMotor.rotate(convertAngle(rWheel, dBase, turnTheta));

	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width);
	}
	private static void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}
	private static void turnTo(double destTheta) //uses destTheta and nowTheta to calculate AND TURN required minimal angle.
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
		rightMotor.rotate(convertAngle(rWheel, dBase, turnTheta), true);
	}
}
