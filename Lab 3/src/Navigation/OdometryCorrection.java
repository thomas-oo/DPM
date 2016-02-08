/* 
 * OdometryCorrection.java
 */
package Navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 2;
	private long correctionStart,correctionEnd;

	private static Port colorPort = LocalEV3.get().getPort("S1");
	EV3ColorSensor lightSensor = new EV3ColorSensor(colorPort);
	SensorMode lightSamples = lightSensor.getRedMode();
	float[] lightData = new float[lightSamples.sampleSize()];

	private float lightValue;
	private int count;
	private double[] nowPosition = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;
	private double thetaThreshold;

	private Odometer odometer;
	private Navigator nav;

	public OdometryCorrection(Navigator nav, Odometer odometer) {
		this.nav = nav;
		this.odometer = odometer;
	}
	enum State {INIT,ACROSS,ALONG,INTERSECTION};
	public void run(){ 
		while(true)
		{
			correctionStart = System.currentTimeMillis();
			lightSamples.fetchSample(lightData,0);	//fetching the sample data						
			lightValue=lightData[0];
			if(lightValue<0.3 && count<7) //buffer
			{
				count ++;
			}

			else if (lightValue<0.3 && nav.isCorrecting()) //passed buffer, black line detected
			{
				Sound.beep();
				odometer.getPosition(nowPosition, new boolean[] { true, true, true });
				nowX = nowPosition[0];
				nowY = nowPosition[1];
				nowTheta = nowPosition[2];

				//do correct under these situations
				//when crossing horizontal lines
				//when crossing vertical lines
				//when travel ALONG a horizontal/vertical line
				//do not correct under these situations
				//when crossing an intersection
				//when not facing in the destination (avoids the case where correction is wrong when correcting DURING an oscillilation/turning)
				State state = State.INIT;
				switch(state)
				{
				case INIT:
					if(checkIfIntersection() == true) //implemented.
					{
						state = state.INTERSECTION;
					}
					else if(checkIfAlong() == true)
					{
						state = state.ALONG;
					}
					else
					{
						state = state.ACROSS;
					}
				case ACROSS:
					if(nowX < 30 || nowY < 30) //will not correct for first square due to complications
					{
					}
					else if(Math.abs(nowX)%30< 1) //if nowX is close to a multiple of 30, it is a vertical line, correct to the nearest multiple of 30
					{
						odometer.setX(30*Math.floor(nowX/30));
					}
					else if(Math.abs(nowY)%30 < 1) //if nowY is close to a multiple of 30, it is a horizontal line, correct to the nearest multiple of 30
					{
						odometer.setY(30*Math.floor(nowY/30));
					}
					break;
				case ALONG:
					if(nowX < 30 || nowY < 30) //will not correct for first square due to complications
					{
					}
					else if(Math.abs(nowX)%30< 1) //if nowX is close to a multiple of 30, it is a vertical line, correct to the nearest multiple of 30
					{
						odometer.setX(30*Math.floor(nowX/30));
					}
					else if(Math.abs(nowY)%30 < 1) //if nowY is close to a multiple of 30, it is a horizontal line, correct to the nearest multiple of 30
					{
						odometer.setY(30*Math.floor(nowY/30));
					}
					break;
				case INTERSECTION:
					break;
				}

				if (lightValue>0.3)
				{
					count = 0;
				}

				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
					try {
						Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here because it is not
						// expected that the odometry correction will be
						// interrupted by another thread
					}
				}
			}
		}

	}
	private boolean checkIfIntersection() 
	{
		if(Math.abs(nowX)%30 < 1 && Math.abs(nowY)%30 < 1)
		{
			return true;
		}
		else
			return false;
	}
	private boolean checkIfAlong() 
	{
		if(Math.abs(nowTheta%(0.5*Math.PI)) < thetaThreshold)
		{
			return true;
		}
		return false;
	} 
}



