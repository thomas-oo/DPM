package Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidance extends Thread
{
	Navigator nav;
	boolean safe;
	double pastX, pastY, idealTheta;
	double calcX, calcY, calcTheta;
	
	private Odometer odometer;
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;

	private double[] avoidanceNowDistance = new double[3];
	private double avoidanceNowX,avoidanceNowY;
	private double distThreshold = 0.5;
	private double thetaThreshold = 0.0078565804;
	

	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private SampleProvider sampleProvider;
	private float[] usData;

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
		while(!safe)
		{
			int distance;
			sampleProvider.fetchSample(usData,0);
			distance=(int)(usData[0]*100.0);
			
			odometer.getPosition(avoidanceNowDistance, new boolean[]{true, true, true});
			avoidanceNowX = avoidanceNowDistance[0];
			avoidanceNowY = avoidanceNowDistance[1];

			calcX = avoidanceNowX - pastX;
			calcY = avoidanceNowY - pastY;
			
			calcTheta = Math.atan(calcY/calcX);
			
			calcTheta = convertTheta(calcTheta);
			
			
			if (Math.abs(calcTheta- idealTheta) <= thetaThreshold)
			{
				if(Math.abs(avoidanceNowX - pastX) < distThreshold && Math.abs(avoidanceNowY - pastY) < distThreshold) //
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
	public int readUSDistance() {
		return this.readUSDistance();
	}
	public boolean getSafe()
	{
		return this.safe;
	}
}