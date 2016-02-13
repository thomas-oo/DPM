package Localization;


import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.UARTPort;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import Localization.LCDInfo;

public class Lab4 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");		
	private static final Port colorPort = LocalEV3.get().getPort("S1");
	public static double rWheel = 2.1; //give values
	public static double dBase = 15.7;
	public static int bandCenter = 20;
	public static int bandWidth = 3;
	public static int motorLow = 100;
	public static int motorHigh = 200;

	
	public static void main(String[] args) throws InterruptedException {
		
		
/*		int option = 0;
		while (option == 0)								// and wait for a button press.  The button
			option = Button.waitForAnyPress();			// ID (option) determines what type of control to use
		*/
		@SuppressWarnings("resource")							    	
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];				
		
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];			
				
		Odometer odo = new Odometer(rWheel, dBase);
		odo.start();
		
/*		switch(option) {
		case Button.ID_LEFT:										// Bang-bang control selected
			USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE, leftMotor, rightMotor);
			LCDInfo lcd = new LCDInfo(odo,usl);
			usl.doLocalization();
			break;
		case Button.ID_RIGHT:										// Proportional control selected
			USLocalizer usl2 = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.RISING_EDGE, leftMotor, rightMotor);
			LCDInfo lcd2 = new LCDInfo(odo,usl2);
			usl2.doLocalization();
			break;
		default:
			System.out.println("Error - invalid button");			// None of the above - abort
			System.exit(-1);
			break;
		}*/
		
		USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE, leftMotor, rightMotor);
		LCDInfo lcd = new LCDInfo(odo,usl);
		usl.doLocalization();
		
		
		LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData);
		lsl.doLocalization();
		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}

