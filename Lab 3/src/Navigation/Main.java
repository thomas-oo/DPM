package Navigation;

import Navigation.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main 
{
	//Declare variables
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");
	public static double rWheel = 0; //measure
	public static double dBase = 0;
	public static int bandCenter = 0;
	public static int bandWidth = 0;
	private static Navigator nav;
	
	public static void main(String[] args) throws InterruptedException
	{
		PController pControl = new PController(leftMotor, rightMotor, bandCenter, bandWidth);
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance

		SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance

		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
		
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, pControl);							// the selected controller on each cycle
		
		Odometer odometer = new Odometer(leftMotor, rightMotor, rWheel, dBase);
		odometer.start(); 
		Navigator nav = new Navigator(odometer, leftMotor, rightMotor, usPoller);
		nav.start();
		completeCourse();
		
		
	}
	private static void completeCourse() throws InterruptedException
	{
		int[] [] waypoints = {{60,30}, {30,30}, {30,60}, {60,0}};
		for(int [] point : waypoints)
		{
			nav.travelTo(point[0], point[1]);
			while(nav.isTravelling())
			{
				Thread.sleep(500);
			}
		}
	}
}
