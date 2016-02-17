package Localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread
{
	private int forwardSpeed = 100;
	
	//destination variables
	private double[] destDistance = new double[2];
	private double destTheta;
	
	//current coordinates
	private double[] nowDistance = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;
	
	//thresholds to allow control to within this window.
	private double thetaThreshold = 0.0349066; //2 degrees in rads
	private double destThreshold = 0.25; //0.5 cm of deviation allowed in navigating to waypoint
	
	//state variables
	private boolean isNavigating;
	private boolean isCorrecting;
	
	//motors that need to be used
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	//classes that navigator depends on
	private Odometer odometer;
	

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer)
	{
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	enum State {INIT, TURNING, TRAVELING};

	public void run()
	{
		//3 states
			//state for initializing
				//if navigator has a destination, go to state turning
			//state for turning
				//turn to the destination theta if it isn't already facing that direction
					//go to state traveling
			//state for traveling
				//check if you are already at the waypoint, 
					//if not, move with forward speed
						//for the next cycle, recalc a the latest destination theta with latest coordinates from odometer
						//on next cycle, go to state init to make sure that you are heading in the new theta direction
					//if so, stop, and set the flag isNavigating to false, lets the main method know that the waypoint is reached
		State state = State.INIT;
		while (isNavigating)
		{
			odometer.getPosition(nowDistance, new boolean[]{true, true, true});
			nowX = nowDistance[0];
			nowY = nowDistance[1];
			nowTheta = nowDistance[2];
		
			isCorrecting = false;
			
			switch(state)
			{
			case INIT:
				System.out.println("INIT.");
				if(isNavigating)
				{
					state = State.TURNING;
				}
				break;
			case TURNING:
				System.out.println("TURNING.");
				isCorrecting = false;
				if (!facingDest(destTheta))
				{
					leftMotor.stop();
					rightMotor.stop();
					turnTo(destTheta);
				}
				else if(facingDest(destTheta))
				{
					state = State.TRAVELING;
				}
				break;
			case TRAVELING:
				System.out.println("TRAVELING");
				isCorrecting = true;
/*				if(checkEmergency())
				{
					state = State.WALL;
					break;
				}*/
				if(!checkIfDone(nowDistance))
				{
					setSpeeds(forwardSpeed, forwardSpeed);
					leftMotor.forward();
					rightMotor.forward();
					updateTravel();
					state = State.INIT;
				}
			    else if(checkIfDone(nowDistance))
				{
					leftMotor.stop();
					rightMotor.stop();
					System.out.println("Arrived.");
					isNavigating = false;
					state = State.INIT;
				}
				break;
			}
			try
			{
				Thread.sleep(30);
			}
			catch(InterruptedException e)
			{
				e.printStackTrace();
			} 	
		}
	}

	private void updateTravel() //update the destination angle of the next "cycle" of travel
	{
		destTheta = getDestAngle();
	}
	private boolean facingDest(double destTheta) //checks if nowTheta is facing destTheta within a leeway of thetaThreshold 
	{
		if(nowTheta > (destTheta - thetaThreshold) && nowTheta < (destTheta + thetaThreshold))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	private boolean checkIfDone(double[] nowDistance) //checks if nowX, Y are within destThreshold of destDistnace[0], [1]
	{
		if(nowX > (destDistance[0]-destThreshold) && (nowX < (destDistance[0] + destThreshold)))
		{
			if(nowY > (destDistance[1]-destThreshold) && (nowY < (destDistance[1]+ destThreshold)))
			{
				return true;
			}
			return false;
		}
		return false;
	}
	private void setSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		isNavigating();
	}
	public void travelTo(double destX, double destY) //called from main, (never called inside navigator). Is basically a setter for destination and flags isNavigating to true
	{
		destDistance[0] = destX;
		destDistance[1] = destY;
		destTheta = getDestAngle();
		isNavigating = true;
	}
	private double getDestAngle() //uses destDistance[0],[1] to calculate destTheta. destTheta is the heading with respect to coordinate system. 
								  //(where the min is 0 (on positive x-axis), and max is 2pi. positive theta is counterclockwise angle from the positive x-axis.
	{
		double errorX = destDistance[0] - nowX;
		double errorY = destDistance[1] - nowY;

		if(Math.abs(errorX) < destThreshold)
		{
			if(errorY > 0)
			{
				return 0.5 * Math.PI; //90
			}
			else
			{
				return 1.5 * Math.PI; //270
			}
		}
		else if(Math.abs(errorY) < destThreshold)
		{
			if(errorX > 0)
			{
				return 0.0; //0
			}
			else
			{
				return Math.PI; //180
			}
		}

		
		else if(errorX > 0) 
		{
			if(errorY > 0) //positive theta
			{
				return Math.atan(errorY/errorX);
			}
			else //converts quadrant 4 into a positive theta
			{
				return 2*Math.PI + Math.atan(errorY/errorX);
			}
		}
		else if(errorX < 0)
		{
			if(errorY > 0) //quad 2, positive theta
			{
				return (Math.atan(errorY/errorX) + Math.PI);
			}
			else if(errorY < 0) //quad 3, positive theta
			{
				return (Math.atan(errorY/errorX) + Math.PI);
			}
		}
		return 0; //IF SOMETHING GOES HORRIBLY WRONG, RETURN 0.
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
		}
		else if(turnTheta>Math.PI && turnTheta < 2*Math.PI)
		{
			turnTheta = turnTheta - 2*Math.PI;
		}
		else
		{
			System.out.println("turnTheta error: " + turnTheta);
		}
		leftMotor.rotate(-convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta), true);
		rightMotor.rotate(convertAngle(Lab4.rWheel, Lab4.dBase, turnTheta));
	}
	public boolean isNavigating() //returns true if another thread has called travelTo or turnTo
	{
		return isNavigating;
	}
	public boolean isCorrecting() //returns true if navigator is currently turning
	{
		return isCorrecting;
	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((90.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width);
	}
	public void setNavigating(boolean isNavigating) //sets isNavigating if ever needed (unused here)
	{
		this.isNavigating = isNavigating;
	}
}