package Navigation;

import lejos.robotics.SampleProvider;

public class UltrasonicPoller {

	private SampleProvider us;
	private float[] usData;
	private PController pControl;
	
	public UltrasonicPoller(SampleProvider us, float[] usData, PController pControl) {
		this.us = us;
		this.pControl = pControl;
		this.usData = usData;
	}

//  Sensors now return floats using a uniform protocol.
//  Need to convert US result to an integer [0,255]
	
	public void run() {
		int distance;
		while (true) {
			us.fetchSample(usData,0);							// acquire data
			distance=(int)(usData[0]*100.0);					// extract from buffer, cast to int
			pControl.processUSData(distance);						// now take action depending on value
			try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	
}
