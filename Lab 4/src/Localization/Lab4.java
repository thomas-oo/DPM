package Localization;

import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab4 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S2");		
/*	private static final Port colorPort = LocalEV3.get().getPort("S2");*/
	public static double rWheel = 2.1; //give values
	public static double dBase = 15.7;

	
	public static void main(String[] args) {
		@SuppressWarnings("resource")							    	
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];				
		
/*		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];	*/		
				
		Odometer odo = new Odometer(rWheel, dBase);
		odo.start();
		LCDInfo lcd = new LCDInfo(odo);
		
		USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE, leftMotor, rightMotor);
		usl.doLocalization();
		
/*		LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData);
		lsl.doLocalization();*/			
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
