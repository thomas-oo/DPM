package Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	private double x, y, theta;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int oldTachoLeft, oldTachoRight, nowTachoLeft, nowTachoRight;
	private double d1, d2, dh, d;
	private double rWheel, dBase;

	private static final long ODOMETER_PERIOD = 25;

	private Object lock;

	public Odometer(double rWheel, double dBase) {
		x = 0.0;
		y = 0.0;
		theta = 0.5*Math.PI;
		this.leftMotor = Main.leftMotor;
		this.rightMotor = Main.rightMotor;
		this.rWheel = rWheel;
		this.dBase = dBase;
		oldTachoLeft = 0;
		oldTachoRight = 0;
		nowTachoLeft = 0;
		nowTachoRight = 0;
		lock = new Object();
	}

	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			nowTachoLeft = leftMotor.getTachoCount();
			nowTachoRight = rightMotor.getTachoCount();
			synchronized (lock)
			{
				d1 = Math.PI*rWheel*(nowTachoLeft - oldTachoLeft)/180;
				d2 = Math.PI*rWheel*(nowTachoRight - oldTachoRight)/180;
				oldTachoLeft = nowTachoLeft;
				oldTachoRight = nowTachoRight;
				dh = 0.5*(d1 + d2);
				d = d2 - d1;
				theta += d/dBase;
				if(theta < 0)
				{
					theta += 2 * Math.PI;
				}
				if(theta > 2*Math.PI)
				{
					theta -= 2 * Math.PI;

				}
				x += dh * Math.cos(theta);
				y += dh * Math.sin(theta);
			}
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try 
				{
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} 
				catch (InterruptedException e) {
				}
			}
		}
	}
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
