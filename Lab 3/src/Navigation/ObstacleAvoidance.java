package Navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidance extends Thread
{ //this class will run until the robot reaches where it's supposed to be to continue trajectory
	Navigator nav;
	boolean safe;
	double pastX, pastY, idealTheta;
	double calcX, calcY, calcTheta;
	
	private Odometer odometer;
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;

	private double[] avoidanceNowDistance = new double[3];
	private double avoidanceNowX,avoidanceNowY,avoidanceNowTheta ;
	private double distThreshold = 0.5;
	private double thetaThreshold = 0.0078565804;
	

	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private SampleProvider sampleProvider;
	private float[] usData;
	private double usDistance;
	private SensorModes usSensor;

	public ObstacleAvoidance(Navigator nav,double pastX, double pastY, double idealTheta, Odometer odometer,EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandWidth, int motorLow, int motorHigh, SampleProvider sampleProvider)
	{
		this.odometer = odometer;
		this.nav = nav;
		this.safe = false;
		this.pastX = pastX;
		this.pastY = pastY;
		this.idealTheta = idealTheta;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.bandCenter = bandCenter;
		this.bandwidth = bandWidth;
		this.motorLow = motorLow; 
		this.motorHigh = motorHigh;
		
		this.sampleProvider = sampleProvider;
		this.usData = new float[sampleProvider.sampleSize()];

	}
	public void run()
	{
		while(!safe)// run this loop until it becomes safe AKA, at the position it's supposed to be at
		{
			int distance;
			sampleProvider.fetchSample(usData,0);							// acquire data
			distance=(int)(usData[0]*100.0); //distance in cm?	

			//BangBangController bangbang = new BangBangController(leftMotor, rightMotor,
			//bandCenter, bandwidth, motorLow, motorHigh);

			odometer.getPosition(avoidanceNowDistance, new boolean[]{true, true, true}); //get pos.
			avoidanceNowX = avoidanceNowDistance[0];
			avoidanceNowY = avoidanceNowDistance[1];
			avoidanceNowTheta = avoidanceNowDistance[2]; //cannot use this, won't calculate the same theta, this is practically useless

			calcX = avoidanceNowX - pastX;
			calcY = avoidanceNowY - pastY;
			
			calcTheta = Math.atan(calcY/calcX);
			
			calcTheta = convertTheta(calcTheta);
			
			
			if (Math.abs(calcTheta- idealTheta) <= thetaThreshold) //maybe change a bit? avoidY-pastY , etc?
			{
				if(Math.abs(avoidanceNowX - pastX) < distThreshold || Math.abs(avoidanceNowX - pastX) < distThreshold)
				{
					processUSData(distance);
				}
				else
				{
					safe = true;
				}
			}
			else
			{
				processUSData(distance);
			}
		}
		//when the safe is true 	
	}


	private double convertTheta(double calcTheta2) 
	{
		if(calcX > 0) 
		{
			if(calcY > 0) //positive theta
			{
				return Math.atan(calcY/calcX);
			}
			else //converts quadrant 4 into a positive theta
			{
				return 2*Math.PI + Math.atan(calcY/calcX);
			}
		}
		else if(calcX < 0)
		{
			if(calcY > 0) //quad 2, positive theta
			{
				return (Math.atan(calcY/calcX) + Math.PI);
			}
			else if(calcY < 0) //quad 3, positive theta
			{
				return (Math.atan(calcY/calcX) + Math.PI);
			}
		}
		else if(Math.abs(calcX) < distThreshold)
		{
			if(calcY > 0)
			{
				return 0.5*Math.PI;
			}
			else
			{
				return 1.5*Math.PI;
			}
		}
		else if(Math.abs(calcY) < distThreshold)
		{
			
			if(calcX > 0)
			{
				return 0;
			}
			else
			{
				return Math.PI;
			}
			
		}
		return 0.0; //if all else goes wrong
	}
	public void processUSData(int distance) { //this is bang bang
		/*		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			// gives robot time to make a sharper turn
			filterControl ++;
		} 
		else if (distance >= 255){
			// true 255, therefore set distance to 255
			//also move closer slowly so you can detect a wall
			this.distance = 255;
			leftMotor.setSpeed(motorHigh - motorLow); //left motor moves slower
			rightMotor.setSpeed(motorHigh + motorLow); //right motor moves faster
			leftMotor.forward();
			rightMotor.forward();

		} 
		else if (distance < 255)
		{*/
		int difference = distance - bandCenter;
		if (Math.abs(difference)<=(bandwidth)){
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}


		//if EV3 is too far from the wall
		else if ((difference)>0){
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}

		//if Ev3 is too close to the wall
		else if ((difference)<0){
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorLow);
			leftMotor.forward();
			rightMotor.forward();
		}
	}


	//go straight if less than bandwidth 
	public int readUSDistance() {
		return this.readUSDistance();
	}
	
	public boolean getSafe()
	{
		return this.safe;
	}
}