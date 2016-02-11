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

	private double usValue;
	private int forwardSpeed = 30;
	private double alpha;
	private double beta;

	private double nowTheta;
	private double distance;

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

	public void doLocalization() 
	{
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
					while (usValue < 50)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					leftMotor.stop();
					rightMotor.stop();
					state = State.FIRSTWALL;
					break;
				case FIRSTWALL:
					while (usValue>50)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					leftMotor.stop();
					rightMotor.stop();
					alpha = odo.getTheta();
					state= State.SECONDWALL;
					break;
				case SECONDWALL:
					while (usValue<50) //this needs to be changed
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					while (usValue>50)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					leftMotor.stop();
					rightMotor.stop();
					beta = odo.getTheta();
					done = true;
					break;
				}
			}
			if (faceWall) //A>B
			{
				deltaTheta = 1.5*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta +beta);
				turnTo(0);
			}
			else//A<B
			{
				deltaTheta = 0.5*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta +beta);
				turnTo(0);
			}
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		} 
		else if(locType == LocalizationType.RISING_EDGE)
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
						rotateCounterClockwise();
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
						rotateCounterClockwise();
						usValue = getFilteredData();
					}
					//once we don't find a wall (rising edge), we are at alpha
					leftMotor.stop();
					rightMotor.stop();
					alpha = odo.getTheta(); //get the bounded degree reading of theta from the odometer and set to alpha.
					state = State.SECONDWALL; //ready to find beta now
					break;
				case SECONDWALL: //find beta
					usValue = getFilteredData();
					while(usValue>50) //this is here in case we overshot alpha, and now have to turn clockwise to find the wall again
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					while(usValue<50) //we rotate clockwise until we DON'T see a wall
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					//once we don't find a wall (rising edge), we are at beta
					leftMotor.stop();
					rightMotor.stop();
					beta = odo.getTheta(); //get the bounded degree reading of theta from the odometer and set to beta.
					done = true; //we have now found beta AND alpha, we are "done"
					break;
				}
			}
			//we get to this part of the code once done = true;
			if (faceWall) // A>B
			{
				deltaTheta = 0.5*Math.PI - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			else //A<B
			{
				deltaTheta = 1.5*Math.PI - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		}
	}

	private void rotateCounterClockwise() {
		leftMotor.setSpeed(-forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}

	private void rotateClockwise() {
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(-forwardSpeed);
		leftMotor.forward();
		rightMotor.forward();
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
		nowTheta = odo.getTheta();

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
	public double getFilteredData() {
		usSensor.fetchSample(usData, 0);
		distance = (double)(usData[0]*100.0);
		if(distance > 50)
		{
			distance = 50;
		}
		else if(distance < 0)
		{
			distance = 50;
		}
		return distance;
	}

}


