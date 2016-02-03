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
	private double destX;
	private double destY;
	private double destTheta; //max is 359, min is 0
	
	private double nowX;
	private double nowY;
	private double nowTheta; //max is 359, min is 0
	
	//Motors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	//Constructor
	public Navigator(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) //not sure what to pass to constructor yet..
	{
		
	}
	
	public void run()
	{
		while (true) //forward and angular error calculator.
		{
			//C O R E:
			//call odometer getters to get nowX, nowY, nowTheta
			//use destX and destY and nowX and nowY to calculate error
			//use error to calculate destTheta
			//pass destTheta to turnTo
			
			//once turned, 
		}
	}
	private void travelTo(double x, double y)
	{
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
	private void turnTo(double theta)
	{
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
	private boolean isTravelling() 
	//returns true if another thread has called travelTo or turnTo
	{
		return true;
	}

}
