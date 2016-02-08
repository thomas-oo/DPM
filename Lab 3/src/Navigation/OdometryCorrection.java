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
	private double thetaThreshold = 0.034906585;

	private Odometer odometer;
	private Navigator nav;

	public OdometryCorrection(Navigator nav, Odometer odometer) {
		this.nav = nav;
		this.odometer = odometer;
	}
	enum State {INIT,ACROSS,ALONG,INTERSECTION};
	public void run(){ 
		State state = State.INIT;
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
				switch(state)
				{
				case INIT:
					if(checkIfIntersection()) //implemented.
					{
						state = state.INTERSECTION;
					}
					else if(checkIfAlong())
					{
						state = state.ALONG;
					}
					else
					{
						state = state.ACROSS;
					}
					System.out.println("Intersection: " + checkIfIntersection());
					System.out.println("Along: " + checkIfAlong());
					break;
				case ACROSS:
					if(nowX < 30 || nowY < 30) //will not correct for first square due to complications
					{
						if(Math.abs(30-nowX) < 1)
						{
							odometer.setX(30);
							System.out.println("Across: corrected X to 30");
						}
						else if(Math.abs(30-nowY) < 1)
						{
							odometer.setY(30);
							System.out.println("Across: corrected Y to 30");
						}
						else if(Math.abs(nowX) < 1)
						{
							odometer.setX(0);
							System.out.println("Across: corrected X to 0");
						}
						else if(Math.abs(nowY) < 1)
						{
							odometer.setY(0);
							System.out.println("Across: corrected Y to 0");
						}
					}
					else if(Math.abs(nowX)%30< 1) //if nowX is close to a multiple of 30, it is a vertical line, correct to the nearest multiple of 30
					{
						odometer.setX(30*Math.floor(nowX/30));
						System.out.println("Across: corrected X");
					}
					else if(Math.abs(nowY)%30 < 1) //if nowY is close to a multiple of 30, it is a horizontal line, correct to the nearest multiple of 30
					{
						odometer.setY(30*Math.floor(nowY/30));
						System.out.println("Across: corrected Y");
					}
					state = state.INIT;
					break;
				case ALONG:
					if(nowX < 30 || nowY < 30) //will not correct for first square due to complications
					{
					}
					else if(Math.abs(nowX)%30< 1) //if nowX is close to a multiple of 30, it is a vertical line, correct to the nearest multiple of 30
					{
						odometer.setX(30*Math.floor(nowX/30));
						System.out.println("Along: corrected X");
					}
					else if(Math.abs(nowY)%30 < 1) //if nowY is close to a multiple of 30, it is a horizontal line, correct to the nearest multiple of 30
					{
						odometer.setY(30*Math.floor(nowY/30));
						System.out.println("Along: corrected Y");
					}
					state = state.INIT;
					break;
				case INTERSECTION:
					state = state.INIT;
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
		else
			return false;
	} 
}



