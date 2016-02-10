package Localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 30;

	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private float usValue;
	private int forwardSpeed = 150;
	private double alpha;
	private double beta;

	private double[] nowDistance = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;

	private boolean faceWall; //come up with a better name 
	private double deltaTheta;
	private boolean done; //alpha and betas are determined


	enum State {INITWALL, NOWALL, WALL, FIRSTWALL, SECONDWALL};


	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	public void doLocalization() {
		double [] pos = new double [3];
		double angleA, angleB;

		if (locType == LocalizationType.FALLING_EDGE) 
		{
			usValue = getFilteredData();
			State state = State.INITWALL;
			done = false;
			while(!done)
			{

				switch(state)
				{
				case INITWALL:
					if (usValue>50)
					{
						faceWall = false;
						state= State.FIRSTWALL;
						break;
					}
					else
					{
						faceWall = true;
						state = State.WALL;
						break;
					}
				case WALL:
					// keep rotating until the robot sees a wall, then latch the angle
					while (usValue>50)
					{
						leftMotor.setSpeed(forwardSpeed); // keep rotating clockwise until it sees a wall
						rightMotor.setSpeed(-forwardSpeed);
					}
					alpha = nowTheta;
					state= State.FIRSTWALL;
					break;

				case FIRSTWALL:

					while (usValue<50) //this needs to be changed
					{
						leftMotor.setSpeed(forwardSpeed); // rotate counterclockwise until it sees the second wall
						rightMotor.setSpeed(-forwardSpeed);
					}
					beta = nowTheta;
					state= State.SECONDWALL;
					break;

				case SECONDWALL:

					//case alpha<beta
					if (!faceWall)
					{
						deltaTheta = 45-((alpha+beta)/2);
						odo.setTheta(deltaTheta +beta);
						turnTo(0);
					}

					//	case alpha>beta
					if (faceWall)
					{
						deltaTheta = 225-((alpha+beta)/2);
						odo.setTheta(deltaTheta +beta);
						turnTo(0);
					}

				}	
			}
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		} 
		else 
		{
			//start by figuring out if we started facing the wall or not
			usValue = getFilteredData();
			State state = State.INITWALL;
			done = false; //we are not done figuring out alpha and beta (we will figure alpha THEN beta)

			while (!done) 
			{
				switch (state) 
				{
				case INITWALL:
					if (usValue > 50) 
					{
						faceWall = false;
						state = State.NOWALL;
						break;
					} 
					else 
					{
						faceWall = true;
						state = State.FIRSTWALL;
						break;
					}

				case NOWALL:
					while (usValue > 50) //will turn counterclockwise until we find a wall.
					{
						leftMotor.setSpeed(-forwardSpeed);
						rightMotor.setSpeed(forwardSpeed);
						leftMotor.forward();
						rightMotor.forward();
						usValue = getFilteredData();
					}
					//when we do finally arrive at a wall, we will be facing the left wall (primed to find alpha)
					leftMotor.stop();
					rightMotor.stop();
					state = State.FIRSTWALL;
					break;

				case FIRSTWALL: //find alpha
					usValue = getFilteredData();
					while(usValue < 50) //keep rotating counterclockwise until we DON'T find a wall
					{
						leftMotor.setSpeed(-forwardSpeed);
						rightMotor.setSpeed(forwardSpeed);
						leftMotor.forward();
						rightMotor.forward();
						usValue = getFilteredData();
					}
					//once we don't find a wall (rising edge), we are at alpha
					leftMotor.stop();
					rightMotor.stop();
					alpha = convertToDeg(odo.getTheta()); //get the bounded degree reading of theta from the odometer and set to alpha.
					state = State.SECONDWALL; //ready to find beta now
					break;
				case SECONDWALL: //find beta
					usValue = getFilteredData();
					while(usValue>50) //this is here in case we overshot alpha, and now have to turn clockwise to find the wall again
					{
						leftMotor.setSpeed(forwardSpeed); 
						rightMotor.setSpeed(-forwardSpeed);
						leftMotor.forward();
						rightMotor.forward();
						usValue = getFilteredData();
					}
					while(usValue<50) //we rotate clockwise until we DON'T see a wall
					{
						leftMotor.setSpeed(forwardSpeed);
						rightMotor.setSpeed(-forwardSpeed);
						leftMotor.forward();
						rightMotor.forward();
						usValue = getFilteredData();
					}
					//once we don't find a wall (rising edge), we are at beta
					leftMotor.stop();
					rightMotor.stop();
					beta = convertToDeg(odo.getTheta()); //get the bounded degree reading of theta from the odometer and set to beta.
					done = true; //we have now found beta AND alpha, we are "done"
					break;
				}
			}
			//we get to this part of the code once done = true;
			if (faceWall) // B<A
			{
				deltaTheta = 45 - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			else //B>A
			{
				deltaTheta = 225 - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		}
	}
	private double convertToDeg(double theta) {
		if(theta > 2*Math.PI)
		{
			theta = theta- 2*Math.PI;
		}
		else if(theta < 0)
		{
			theta = theta + 2*Math.PI;
		}
		else
		{
		}
		return theta * 57.2598;
	}

	private void turnTo(double destTheta) //uses destTheta and nowTheta to calculate AND TURN required minimal angle.
	{
		setSpeeds(forwardSpeed, forwardSpeed); //set speeds as we will be moving.

		double turnTheta = destTheta - nowTheta; //dest and nowTheta both are from [0,2pi]
		//CALCULATES MINIMAL TURN and EXECUTES
		//ROTATES UNTIL TURN IS COMPLETE.
		if(turnTheta >= -Math.PI && turnTheta <= Math.PI)
		{
		}
		else if(turnTheta < -Math.PI && turnTheta > -2*Math.PI)
		{
			turnTheta = turnTheta + 2*Math.PI;
			System.out.println("a");
		}
		else if(turnTheta>Math.PI && turnTheta < 2*Math.PI)
		{
			turnTheta = turnTheta - 2*Math.PI;
			System.out.println("b");
		}
		else
		{
			System.out.println("turnTheta error: " + turnTheta);
		}
		leftMotor.rotate(-convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta), true);
		rightMotor.rotate(convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta));
	}

	private void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width);
	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0];

		return distance;
	}

}


