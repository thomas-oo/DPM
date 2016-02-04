package Navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class DataAcquisition implements TimerListener {
	// Class Constants
	
		public static final int SINTERVAL=101;		// Sampling interval (mS)
		public static final int NSAMPLES=100;		// Number of samples to acquire
		public static final int TIMEOUT=40000;		// Fail if no comms by this time (mS)
		public static final int FWDSPEED=15;		// Forward motion speed (deg/sec)
		public static final int SLEEPINT=500;		// Main thread sleeps for 500 (mS)
		
	// Class variables (persistent)
		
		public static int numSamples;
		public static int currentSample;
		
	// Set up instances of the Text Display, left and right Motors

			static TextLCD t = LocalEV3.get().getTextLCD();
			static RegulatedMotor leftMotor = Motor.A;
			static RegulatedMotor rightMotor = Motor.D;
			
	// Allocate ports for the Color and Touch sensors.
			
			static Port portColor = LocalEV3.get().getPort("S1");
			
	// Attach instances of Color and Touch sensors to specified ports.
			
			static SensorModes myColor = new EV3ColorSensor(portColor);
			
	// Get an instance of a sample provider for each sensor.  Operating
	// modes are specified in the constructor.  Note that the color
	// sensor is set to return the intensity of the reflected light.
			
			static SampleProvider myColorSample = myColor.getMode("Red");
			
	// Need to allocate buffers for each sensor

			static float[] sampleColor = new float[myColor.sampleSize()];
			static float[] sampled = new float[myColor.sampleSize()];
			
	// Entry point
			
			public static void main(String[] args) throws InterruptedException {
				
				boolean rolling = true;			// cart moves while true
				int status;
				numSamples=0;
				
	// Set up display area
				
				t.clear();
				t.drawString("Data Acquisition Demo",0,0,false);
				t.drawString("Remote stream...",0,2,false);
				t.drawString("# Samples ",0,4,false);
				t.drawString("Last Val. ",0,5,false);
				
	// Set up timer interrupts
				
				Timer myTimer = new Timer(SINTERVAL,new DataAcquisition());
				
	// Start cart rolling...
				
				leftMotor.setSpeed(FWDSPEED);
				rightMotor.setSpeed(FWDSPEED);
				leftMotor.forward();
				rightMotor.forward();
				
				if (numSamples<101)
				{
	// Enable exception handler
				
				myTimer.start();
				
	// The Main thread continually updates the display
	// and checks for stop
				
				while(rolling) {
					status=Button.readButtons();
					if ((status==Button.ID_ENTER)||(numSamples>=NSAMPLES)) {
						System.exit(0);
					}
					t.drawInt(numSamples,4,11,4);		// show current count
					t.drawInt(currentSample,4,11,5);	// and current value read
					Thread.sleep(SLEEPINT);				// sleep 'till next cycle
				}
				}
				
				else
				{
					System.out.println("got in the loop");
					//discrete derivative data
					int N = numSamples;
					double[] derivedData = new double[numSamples];
					
						for (int i =2; i<N-1;i++)
						{
							derivedData[i] = (sampled[i]-sampled[i+1]);
							System.out.println(derivedData[i]);
						}
						
					//used the derived data to cancel the noise?? selectively???
						//should everything be zero? except the real data? but then can't we just not cancel the noise?
					
					//find the zero crossing == black line, if there's no zero crossing, then there is no black line	
				}
				

			}

		@Override
		public void timedOut() { //this is called everytime timer fires

	// Acquire sample and write immediately to remote console.

			myColorSample.fetchSample(sampleColor,0);
			//store the sample value collected inside an array, or is that already done?
			sampled [numSamples] = sampleColor[0];
			numSamples++;
			System.out.println(sampleColor[0]*1000);
			
		}
}
