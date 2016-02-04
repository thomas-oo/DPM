package Navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


//CLASS THAT ACTUALLY ROTATES MOTORS
//note: timerlistener (as mentioned in instructions is basically just a timer given to a thread to run a cycle. it the time is over, switch task
public class Navigator extends Thread
{
	//Declare variables
	//needs variables to store destination (x,y)
	//needs variables to store theta that needs to be turned
	//
	private double[] destDistance = new double[2];
	private double destTheta; //max is 359, min is 0
	
	private double[] nowDistance;
	private double nowX;
	private double nowY;
	private double nowTheta; //max is 359, min is 0
	
	private boolean isNavigating;
	
	private Odometer odometer;
	
	private boolean caseUpdate;
	
	//Motors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	
	//Constructor
	public Navigator(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) //not sure what to pass to constructor yet..
	{
		this.odometer = odometer;
	}
	
	enum State {INIT, TURNING, TRAVELLING};
	
	public void run()
	{
		State state = State.INIT;
		while (true) //forward and angular error calculator.
		{
			switch(state)
			{
			case INIT:
				if(isNavigating)
				{
					state = State.TURNING;
					caseUpdate = true;
				}
				break;
			case TURNING:
				turnTo(destTheta);
				if(facingDest(destTheta))
				{
					state = State.TRAVELLING;
					caseUpdate = true;
				}
				break;
			case TRAVELLING:
				odometer.getPosition(nowDistance, null);
				if(!checkIfDone(nowDistance))
				{
					updateTravel();
					caseUpdate = true;
				}
				else //Arrived
				{
					setSpeeds(0,0);
					isNavigating = false;
					state = State.INIT;
					caseUpdate = false;
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
			//C O R E:
			//call odometer getters to get nowX, nowY, nowTheta
			//use destX and destY and nowX and nowY to calculate error
			//use error to calculate destTheta
			//pass destTheta to turnTo
			//once turned, 
		}
	}
	private boolean facingDest(double destTheta) {
		if(destTheta == nowTheta)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	private boolean checkIfDone(double[] nowDistance) {
		// TODO Auto-generated method stub
		
		if(nowDistance[0] == destDistance[0])
		{
			if(nowDistance[1] == destDistance[1])
			{
				return true;
			}
			return false;
		}
		return false;
	}
	private void setSpeeds(int leftSpeed, int rightSpeed) 
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		
	}
	private void updateTravel() {
		// TODO Auto-generated method stub
		
	}
	public void travelTo(double x, double y)
	{
		destDistance[0] = x;
		destDistance[1] = y;
		destTheta = getDestAngle();
		isNavigating = true;
		
		//C O R E:
		
		//use nowX/Y and destX/Y to determine destTheta
		//use nowTheta and destTheta to determine thetaTurn
		//pass thetaTurn to turnTo
		//now set motors to speed up to motorStraight
		//at the same time, repeatedly poll odometer to see if nowX/nowY are approaching destX/Y
			//if so, keep running
			//if not, correction/recalculation?
		//when nearing destX/Y, slow down??
		//setup a threshold to allow some deviation from destX/Y
	}
	private double getDestAngle() //use destX and Y to calculate
	{
		//this works if errorX>0
		
		double errorX = destDistance[0] - nowDistance[0];
		double errorY = destDistance[1] - nowDistance[1];
		
		if(errorX == 0)
		{
			if(errorY > 0)
			{
				return Math.PI;
			}
			else
			{
				return -(Math.PI)/2;
			}
		}
		
		if(errorY == 0)
		{
			if(errorX > 0)
			{
				return 0;
			}
			else
			{
				return Math.PI;
			}
		}
		
		if(errorX > 0)
		{
			return Math.atan(errorY/errorX);
		}
		else
		{
			if(errorY > 0)
			{
				return (Math.PI - Math.atan(errorY/errorX));
			}
			else
				return (Math.PI + Math.atan(errorY/errorX));
		}
		
	}

	private void turnTo(double destTheta)
	{
		double turnTheta = destTheta - nowTheta;
		
		if(turnTheta >= -Math.PI && turnTheta <= Math.PI)
		{
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta));
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
		if(turnTheta > -2*Math.PI && turnTheta < -Math.PI)
		{
			turnTheta = turnTheta + Math.PI;
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta));
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
		if(turnTheta>Math.PI && turnTheta<2*Math.PI)
		{
			turnTheta = turnTheta - Math.PI;
			leftMotor.rotate(-convertAngle(Main.rWheel, Main.dBase, turnTheta));
			rightMotor.rotate(convertAngle(Main.rWheel, Main.dBase, turnTheta));
		}
		
		
		//C O R E:
		
		//use theta to determine actual turn (efficient)
		//if theta >= -180 && theta <= 180
			//turn with theta
		//if theta >= -359 && theta < -180
			//turn with theta + 360
		//if theta > 180 && theta <= 359
			//turn with theta - 360
		
		//Considerations:
		
		//this turning has to be done before movement in travelTo starts
		//consider some timer/blocking motors?
		
		//develop turning algorithm based on Lab 2's turning algo
	}
	public boolean isTravelling() 
	//returns true if another thread has called travelTo or turnTo
	{
		return caseUpdate;
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) { //hopefully works
		return convertDistance(radius, angle*width/2);
		
	}

}
