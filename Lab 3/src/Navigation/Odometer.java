package Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
<<<<<<< HEAD
	private double x, y, theta;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int oldTachoLeft, oldTachoRight, nowTachoLeft, nowTachoRight;
	private double d1, d2, dh, d;
	private double rWheel, dBase;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double rWheel, double dBase) {
		x = 0.0;
		y = 0.0;
		theta = 0.0; //start robot in positive x axis
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.rWheel = rWheel;
		this.dBase = dBase;
		oldTachoLeft = 0;
		oldTachoRight = 0;
		nowTachoLeft = 0;
		nowTachoRight = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here (step 1)
			//read tachometers from both motors
			nowTachoLeft = leftMotor.getTachoCount();
			nowTachoRight = rightMotor.getTachoCount();
			synchronized (lock) //wait for code
			{
				// don't use the variables x, y, or theta anywhere but here!
				d1 = Math.PI*rWheel*(nowTachoLeft - oldTachoLeft)/180; //left wheel distance
				d2 = Math.PI*rWheel*(nowTachoRight - oldTachoRight)/180; //right wheel distance
				oldTachoLeft = nowTachoLeft;
				oldTachoRight = nowTachoRight;
				dh = 0.5*(d1 + d2); //distance of base
				d = d2 - d1; //arclength
				theta += d/dBase; //d/rBase is the delta theta
				x += dh * Math.sin(theta);
				y += dh * Math.cos(theta);
			}
=======
		private double x, y, theta;
		private EV3LargeRegulatedMotor leftMotor, rightMotor;
		private int oldTachoLeft, oldTachoRight, nowTachoLeft, nowTachoRight;
		private double d1, d2, dh, d;
		private double rWheel, dBase;

		// odometer update period, in ms
		private static final long ODOMETER_PERIOD = 25;

		// lock object for mutual exclusion
		private Object lock;

		// default constructor
		public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double rWheel, double dBase) {
			x = 0.0;
			y = 0.0;
			theta = 0.0; //start robot in positive x axis
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			this.rWheel = rWheel;
			this.dBase = dBase;
			oldTachoLeft = 0;
			oldTachoRight = 0;
			nowTachoLeft = 0;
			nowTachoRight = 0;
			lock = new Object();
		}
>>>>>>> refs/remotes/origin/Meko

		// run method (required for Thread)
		public void run() {
			long updateStart, updateEnd;

			while (true) {
				updateStart = System.currentTimeMillis();
				// put (some of) your odometer code here (step 1)
				//read tachometers from both motors
				nowTachoLeft = leftMotor.getTachoCount();
				nowTachoRight = rightMotor.getTachoCount();
				synchronized (lock) //wait for code
				{
					// don't use the variables x, y, or theta anywhere but here!
					d1 = Math.PI*rWheel*(nowTachoLeft - oldTachoLeft)/180; //left wheel distance
					d2 = Math.PI*rWheel*(nowTachoRight - oldTachoRight)/180; //right wheel distance
					oldTachoLeft = nowTachoLeft;
					oldTachoRight = nowTachoRight;
					dh = 0.5*(d1 + d2); //distance of base
					d = d2 - d1; //arclength
					theta += d/dBase; //d/rBase is the delta theta
					x += dh * Math.sin(theta);
					y += dh * Math.cos(theta);
				}

				// this ensures that the odometer only runs once every period
				updateEnd = System.currentTimeMillis();
				if (updateEnd - updateStart < ODOMETER_PERIOD) {
					try 
					{
						Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
					} 
					catch (InterruptedException e) {
						// there is nothing to be done here because it is not
						// expected that the odometer will be interrupted by
						// another thread
					}
				}
			}
		}

		// accessors
		// thus getters have synchronized statements to avoid data from changing while method runs
		public void getPosition(double[] position, boolean[] update) {
			// ensure that the values don't change while the odometer is running
			synchronized (lock) 
			{
				if (update[0])
					position[0] = x;
				if (update[1])
					position[1] = y;
				if (update[2])
					position[2] = theta;
			}
		}

		public double getX() {
			double result;

			synchronized (lock) 
			{
				result = x;
			}

			return result;
		}

		public double getY() {
			double result;

			synchronized (lock) {
				result = y;
			}

			return result;
		}

		public double getTheta() {
			double result;

			synchronized (lock) {
				result = theta;
			}

			return result;
		}

		// mutators (setters)
		public void setPosition(double[] position, boolean[] update) {
			// ensure that the values don't change while the odometer is running
			synchronized (lock) {
				if (update[0])
					x = position[0];
				if (update[1])
					y = position[1];
				if (update[2])
					theta = position[2];
			}
		}

		public void setX(double x) {
			synchronized (lock) {
				this.x = x;
			}
		}

		public void setY(double y) {
			synchronized (lock) {
				this.y = y;
			}
		}

		public void setTheta(double theta) {
			synchronized (lock) {
				this.theta = theta;
			}
		}
}
