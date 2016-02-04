package Navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class OdometerCorrection {
<<<<<<< HEAD

		private static final long CORRECTION_PERIOD = 2;
		private Odometer odometer;
		private double[] position = new double[3];
		private double threshold = 4;
		private long correctionStart,correctionEnd;
		private static Port colorPort = LocalEV3.get().getPort("S1");
		private float lightValue;
		private int count;
		private boolean updatedCase1;
		private boolean updatedCase2;


		EV3ColorSensor lightSensor = new EV3ColorSensor(colorPort);
		SensorMode lightSamples = lightSensor.getRedMode();
		float[] lightData = new float[lightSamples.sampleSize()];


		// constructor
		public OdometerCorrection(Odometer odometer) {
			this.odometer = odometer;
		}

		// run method (required for Thread)
		public void run(){ 
			while(true)
			{
				correctionStart = System.currentTimeMillis();
				
//MAKE SURE TO FILTER DATA BEFORE PASSING IT
				lightSamples.fetchSample(lightData,0);	//fetching the sample data						
				lightValue=lightData[0];
				
				if(lightValue<0.3 && count<7) //buffer
				{
					count ++;
				}
				
				else if (lightValue<0.3) //passed buffer, black line detected
				{
					Sound.beep();
					odometer.getPosition(position, new boolean[] { true, true, true });
					
					if ((position[2]*57.2598)<95) //first two runs
					{
						if (Math.abs(position[0]-0)<threshold)	
						{
							if (Math.abs(position[1])<30) //case1
							{
								odometer.setY(15);
								updatedCase1 = true;
								Sound.buzz();
							}
							else if(Math.abs(position[1] - 45) < threshold && (updatedCase1)) //case2, depends on case 1
							{
								odometer.setY(45);
								Sound.buzz();
							}
							//weird cases..
						}
						else if ((position[1])>45 && (position[1])<75)
						{
							if ((position[0])<30) //case3
							{
								odometer.setX(15);
								updatedCase2 = true;
								Sound.buzz();
							}
							else if(Math.abs(position[0] - 45) < (threshold) && (updatedCase2)) //case4, depends on case 3
							{ //changed
								odometer.setX(45);
								Sound.buzz();
							}
						}
					}

					if ((position[2]*57.2598)>=95){
						
						if ((position[0]>45 && (position[0])<75))
						{
							if (Math.abs(position[1]-45)<(threshold)) //case 5
							{
								odometer.setY(45);
								Sound.buzz();
							}
							else if (Math.abs(position[1]-15)<(threshold)) //case 6
							{
								odometer.setY(15);
								Sound.buzz();
							}
						}
						else if (position[1]>-15 && position[1]<15)
						{
							if (Math.abs(position[0]-45)<((threshold+6))) //case 7 changed
							{
								odometer.setX(45);
								Sound.buzz();
							}
							else if (Math.abs(position[0]-15)<(threshold)) //case 8
							{
								odometer.setX(15);
								Sound.buzz();
							}
						}
					}

				}



				//if above 0.08, consider as not black
				if (lightValue>0.3)
				{
					count = 0;
				}


				// this ensure the odometry correction occurs only once every period
				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
					try {
						Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here because it is not
						// expected that the odometry correction will be
						// interrupted by another thread
					}
=======
	private static final long CORRECTION_PERIOD = 2;
	private Odometer odometer;
	private double[] position = new double[3];
	private double threshold = 4;
	private long correctionStart,correctionEnd;
	private static Port colorPort = LocalEV3.get().getPort("S1");
	private float lightValue;
	private int count;
	private boolean updatedCase1;
	private boolean updatedCase2;


	EV3ColorSensor lightSensor = new EV3ColorSensor(colorPort);
	SensorMode lightSamples = lightSensor.getRedMode();
	float[] lightData = new float[lightSamples.sampleSize()];


	// constructor
	public OdometerCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run(){ 
		while(true)
		{
			correctionStart = System.currentTimeMillis();
			
			//MAKE SURE TO FILTER DATA BEFORE PASSING IT
			lightSamples.fetchSample(lightData,0);	//fetching the sample data						
			lightValue=lightData[0];
			
/*			if(lightValue<0.3 && count<7) //buffer
			{
				count ++;
			}
			
			else if (lightValue<0.3) //passed buffer, black line detected
			{
				Sound.beep();
				odometer.getPosition(position, new boolean[] { true, true, true });
				
				if ((position[2]*57.2598)<95) //first two runs
				{
					if (Math.abs(position[0]-0)<threshold)	
					{
						if (Math.abs(position[1])<30) //case1
						{
							odometer.setY(15);
							updatedCase1 = true;
							Sound.buzz();
						}
						else if(Math.abs(position[1] - 45) < threshold && (updatedCase1)) //case2, depends on case 1
						{
							odometer.setY(45);
							Sound.buzz();
						}
						//weird cases..
					}
					else if ((position[1])>45 && (position[1])<75)
					{
						if ((position[0])<30) //case3
						{
							odometer.setX(15);
							updatedCase2 = true;
							Sound.buzz();
						}
						else if(Math.abs(position[0] - 45) < (threshold) && (updatedCase2)) //case4, depends on case 3
						{ //changed
							odometer.setX(45);
							Sound.buzz();
						}
					}
				}

				if ((position[2]*57.2598)>=95){
					
					if ((position[0]>45 && (position[0])<75))
					{
						if (Math.abs(position[1]-45)<(threshold)) //case 5
						{
							odometer.setY(45);
							Sound.buzz();
						}
						else if (Math.abs(position[1]-15)<(threshold)) //case 6
						{
							odometer.setY(15);
							Sound.buzz();
						}
					}
					else if (position[1]>-15 && position[1]<15)
					{
						if (Math.abs(position[0]-45)<((threshold+6))) //case 7 changed
						{
							odometer.setX(45);
							Sound.buzz();
						}
						else if (Math.abs(position[0]-15)<(threshold)) //case 8
						{
							odometer.setX(15);
							Sound.buzz();
						}
					}
				}

			}



			//if above 0.08, consider as not black
			if (lightValue>0.3)
			{
				count = 0;
			}*/


			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
>>>>>>> refs/remotes/origin/Meko
				}
			}
		}
	}
<<<<<<< HEAD
=======
}
>>>>>>> refs/remotes/origin/Meko
