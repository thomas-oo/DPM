package Navigation;

import Navigation.OdometryDisplay;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Main 
{
	//creating instances of motors for use
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor headMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	//specifications about robot, global and should be accessible.
	public static double rWheel = 2.15; //measure
	public static double dBase = 16.4;
	public static int bandCenter = 20;
	public static int bandWidth = 2;
	public static int motorLow = 100;
	public static int motorHigh = 200;
	
	//classes main will rely on
	private static Navigator nav;
	private static Odometer odometer;
	private static OdometryDisplay odometryDisplay;
	private static OdometryCorrection odometryCorrection;
	
	public static void main(String[] args) throws InterruptedException
	{
		odometer = new Odometer(rWheel, dBase);
		final TextLCD t = LocalEV3.get().getTextLCD();
		odometryDisplay = new OdometryDisplay(odometer,t);
		nav = new Navigator(leftMotor, rightMotor, headMotor, odometer);
		odometryCorrection = new OdometryCorrection(nav, odometer);
		
		//starting child threads
		odometer.start(); 
		odometryDisplay.start();
		nav.start();
		odometryCorrection.start();
		
		completeCourse();
	}
	private static void completeCourse() throws InterruptedException
	{
		int [][] waypoints = {{0,60},{60,0}};
		
		//tell navigator to travel to each waypoint. passes waypoints using travlTo method
		for (int[]point:waypoints)
		{
			nav.travelTo(point[0],point[1]);
			while(nav.isNavigating())
			{
				Thread.sleep(500);
			}
		}
	}
}
