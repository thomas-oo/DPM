package Navigation;

import Navigation.OdometryDisplay;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Main 
{
	//Declare variables
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	//private static final Port usPort = LocalEV3.get().getPort("S2");
	public static double rWheel = 2.15; //measure
	public static double dBase = 16.2;
	public static int bandCenter = 20;
	public static int bandWidth = 3;
	public static int motorLow = 100;
	public static int motorHigh = 200;
	private static Navigator nav;
	
	public static void main(String[] args) throws InterruptedException
	{
		//PController pControl = new PController(leftMotor, rightMotor, bandCenter, bandWidth);
		//SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance

		//SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance

		//float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned

		//UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, pControl);							// the selected controller on each cycle

		Odometer odometer = new Odometer(rWheel, dBase);
		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
		nav = new Navigator(leftMotor, rightMotor, odometer);
		BangBangController bangbang = new BangBangController(leftMotor, rightMotor, bandCenter, bandWidth, motorLow, motorHigh);
		
		
		odometer.start(); 
		odometryDisplay.start();
		nav.start();
		completeCourse();
	}
	private static void completeCourse() throws InterruptedException
	{
		int [][] waypoints = {{60,30}, {30,30}, {30,60}, {60,0}};
		for (int[]point:waypoints)
		{
			nav.travelTo(point[0],point[1]); //sets destX and destY to destDistance[0],[1]. as well, sets isNavigating = true
			while(nav.isNavigating())
			{
				Thread.sleep(500);
			}
		}
	}
}
