package localizationtest;

import localizationtest.Odometer;

import java.io.IOException;

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
	public static double[] distData = new double[numSamples]; //please calculate how much space we need.
	public static double[] thetaData = new double[numSamples];
	
	private static int forwardSpeed = 250;

	public static void main(String[] args) throws IOException 
	{
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];

		Odometer odo = new Odometer(rWheel, dBase, samplePeriod);
		odo.start();

		USPoller usPoller = new USPoller(odo, usValue, usData,numSamples, samplePeriod);
		usPoller.start();

		rotate(2*Math.PI);
		System.out.println("THETA: ");
		for(int i = 0; i <thetaData.length; i++)
		{
			System.out.println(thetaData[i]);
		}
		System.out.println("DISTANCE: ");
		for(int i = 0; i <distData.length; i++)
		{
			System.out.println(distData[i]);
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
}
