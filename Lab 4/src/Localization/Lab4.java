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
	
	//Specs of robot
	public static double rWheel = 2.1;
	public static double dBase = 16.2;

	
	public static void main(String[] args) throws InterruptedException {
		
		System.out.println("Press to begin");
		
		Button.waitForAnyPress();
		
		//initialize the ultrasonic sensor
		@SuppressWarnings("resource")							    	
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			
		float[] usData = new float[usValue.sampleSize()];				
		
		//initialize the odometer
		Odometer odo = new Odometer(rWheel, dBase);
		odo.start();
		
		//initialize the ultrasonic localizer
		USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE, leftMotor, rightMotor);
		
		//initialize the display
		LCDInfo lcd = new LCDInfo(odo,usl);
		
		//START ULTRASONIC LOCALIZATION
		usl.doLocalization();
		
		System.out.println("USLOCALIZER DONE");
		System.out.println("Please press a button to initialize lightLocalizer");
		
		Button.waitForAnyPress();
		
		//initialize color sensor
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");			
		float[] colorData = new float[colorValue.sampleSize()];			
		
		//START LIGHT LOCALIZATION
		LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData);
		lsl.doLocalization();
		
		Button.waitForAnyPress();
		System.exit(0);
	}

}

