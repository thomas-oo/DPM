package localizationtest;

import lejos.robotics.SampleProvider;

public class USPoller extends Thread
{
	private SampleProvider us;
	private float[] usData;
	private double distance;
	private long samplePeriod;
	private int index;
	private int numSamples;
	private Odometer odo;
	private double theta;
	
	public USPoller(Odometer odo, SampleProvider us, float[] usData, int numSamples, long samplePeriod) 
	{
		this.us = us;
		this.usData = usData;
		this.numSamples = numSamples;
		this.odo = odo;
		this.samplePeriod = samplePeriod;
	}
	public void run()
	{
		index = 0;
		while (true) 
		{
			us.fetchSample(usData,0);
			distance = (usData[0] * 100.000000000000000);
			if(distance > 200)
			{
				distance = 200;
			}
			theta = odo.getTheta();
			if(index >= numSamples)
			{
				break;
			}
			Main.distData[index] = distance;
			Main.thetaData[index] = theta;
			index++;
			try { Thread.sleep(samplePeriod); } catch(Exception e){} //calculate period based on how many samples you want per rotation
		}
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}	
}
