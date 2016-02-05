package Navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

//note: timerlistener (as mentioned in instructions is basically just a timer given to a thread to run a cycle. it the time is over, switch task
public class Navigator extends Thread
{
	private int forwardSpeed = 150;

	private double[] destDistance = new double[2];
	private double destTheta; //max is 359, min is 0

	private double[] nowDistance = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta; //max is 359, min is 0

	private double thetaThreshold = 0.034906585;
	private double destThreshold = 1;
	
	private boolean isNavigating; //used in state INIT, determines if we will switch state to TURNING //also if false, that means it has arrived.
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private Odometer odometer;
	
	private ObstacleAvoidance avoidance;
	private float[] usData;
	private double usDistance;
	private SampleProvider usSampleProvider;
	private SensorModes usSensor;
	
	private static final Port usPort = LocalEV3.get().getPort("S2");
	

	//Constructor
	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) //not sure what to pass to constructor yet..
	{
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		//constructors to get USsensor data
		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usSampleProvider = usSensor.getMode("Distance");
		this.usData = new float[usSampleProvider.sampleSize()];
	}

	enum State {INIT,WALL,TURNING, TRAVELLING};

	public void run()
	{
		State state = State.INIT;
		//isNavigating will be initialized from main
		while (true) //forward and angular error calculator.
		{
			odometer.getPosition(nowDistance, new boolean[]{true, true, true}); //get pos.
			nowX = nowDistance[0];
			nowY = nowDistance[1];
			nowTheta = nowDistance[2];
			
			
			usSampleProvider.fetchSample(usData, 0); //get latest reading from USsensor.
			usDistance = (double) usData[0];
			
			
			switch(state)
			{
			case INIT:
				if(isNavigating)
				{
					state = State.TURNING;
				}
				break;
			case TURNING:
				if (!facingDest(destTheta))
				{
					leftMotor.stop();
					rightMotor.stop();
					turnTo(destTheta);//turnTo turns until turns are fully complete.
					System.out.println("Not facing destination.");
				}
				else if(facingDest(destTheta))
				{
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				if(checkEmergency())
				{
					state = State.WALL;
					avoidance = new ObstacleAvoidance(this);
					avoidance.start();
				}
				else if(!checkIfDone(nowDistance)) //not there yet
				{
					leftMotor.forward();
					rightMotor.forward();
					updateTravel();
				}
			else (checkIfDone(nowDistance))
				{
					leftMotor.stop();
					rightMotor.stop();
					System.out.println("Arrived.");
					isNavigating = false;//no long navigating, will allow main method to fetch next waypoint
					state = State.INIT; //go back to init. (will not go to turning after as isNavigating is false)
				}
				break;
			case WALL:
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


	private void updateTravel() 
	{
		destTheta = getDestAngle();
	}
	private boolean facingDest(double destTheta) //CHECKS IF NOWTHETA IS FACING DESTTHETA WITHIN THETATHRESHOLD 
	{
		if(nowTheta > (destTheta - thetaThreshold) && nowTheta < (destTheta + thetaThreshold)) //2 degrees of leeway
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	private boolean checkIfDone(double[] nowDistance) //SETS NOWX TO BE NOWDISTANCE0, ETC, CHECKS IF NOWX AND Y ARE WITHIN DESTTHRESHOLD OF DESTDISTANCE[0],[1]
	{
		if((nowX > (destDistance[0] - destThreshold)) && (nowX < (destDistance[0] + destThreshold)))
		{
			if((nowY > (destDistance[1] - destThreshold)) && (nowY < (destDistance[1]+ destThreshold)))
			{
				return true;
			}
			return false;
		}
		return false;
	}
	private void setSpeeds(int leftSpeed, int rightSpeed) //SETS MOTORSPEEDS
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		isNavigating();
	}
	public void travelTo(double destX, double destY) //CALLED FROM MAIN (NEVER CALLED INSIDE NAVIGATOR) (BASICALLY A SETTER)
	{
		destDistance[0] = destX;
		destDistance[1] = destY;
		destTheta = getDestAngle();
		isNavigating = true;
	}
	private double getDestAngle() //USES destDistance[0],[1] TO CALCULATE destTheta. destTheta IS THE HEADING WITH RESPECT TO COORDINATE SYSTEM. (max is 2pi, min is 0)
	{
		double errorX = destDistance[0] - nowX;
		double errorY = destDistance[1] - nowY;

		if(Math.abs(errorX) < destThreshold) //changed 
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
		else if(Math.abs(errorY) < destThreshold) //changed
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
		System.out.println(turnTheta);
		//CALCULATES MINIMAL TURN and EXECUTES
		//ROTATES UNTIL TURN IS COMPLETE.
		if(turnTheta >= -Math.PI && turnTheta <= Math.PI)
		{
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta), true);
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
		if(turnTheta < -Math.PI)
		{
			turnTheta = turnTheta + 2*Math.PI;
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta), true);
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
		if(turnTheta>Math.PI)
		{
			turnTheta = turnTheta - 2*Math.PI;
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta), true);
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
	}
	public boolean isNavigating() 
	//returns true if another thread has called travelTo or turnTo
	{
		return isNavigating;
	}
	private static int convertDistance(double radius, double distance)  //converts linear distance that wheels need to travel into rotations (deg) that the wheels need to perform
	{
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) //converts robot's turn into how much linear distance each wheel needs to travel.
	{ //hopefully works
		return convertDistance(radius, angle*width/2);
	}
	public void setNavigating(boolean isNavigating) //sets isNavigating (unused right now)
	{
		this.isNavigating = isNavigating;
	}
}

