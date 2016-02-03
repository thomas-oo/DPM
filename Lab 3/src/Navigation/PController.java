package Navigation;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController {
	
	private final int bandCenter, bandWidth;
	private final int motorStraight = 200, FILTER_OUT = 60; //old value was 20
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int filterControl;
	private int difference;
	private int absDifference;
	private float ratio;
	private float motorLow;
	
	public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
					   int bandCenter, int bandWidth) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorStraight);					// Initalize motor rolling forward
		rightMotor.setSpeed(motorStraight);
		leftMotor.forward();
		rightMotor.forward();
		filterControl = 0;
		motorLow = 40; //variable that will allow for speed control
	}
	
	public void processUSData(int distance) {
		
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			// gives robot time to make a sharper turn
			filterControl ++;
		} 
		else if (distance >= 255){
			// true 255, therefore set distance to 255
			//also move closer slowly so you can detect a wall
			this.distance = 255;
			leftMotor.setSpeed(motorStraight - motorLow); //left motor moves slower
			rightMotor.setSpeed(motorStraight + motorLow); //right motor moves faster
			leftMotor.forward();
			rightMotor.forward();
			
		} 
		else {
			// distance went below 255, see if the robot is within
			// the bandWidth, very close, close, or far
			filterControl = 0;
			this.distance = distance;
			difference = distance - bandCenter; //used for ratio tuning
			absDifference = Math.abs(difference);
			if(absDifference <= bandWidth) //within bandwidth, go straight
			{
				leftMotor.setSpeed(motorStraight);
				rightMotor.setSpeed(motorStraight);
				leftMotor.forward();
				rightMotor.forward();
			}
			else
			{
				
			ratio = ((float)(absDifference)/(float)(absDifference+bandCenter)); //the ratio to increase/decrease wheel speeds
			
			//the ratios within these if statements were adjusted for our robot, they could be modified otherwise
				if(difference < 0 && difference > -5) //close, move away
				{
					leftMotor.setSpeed(motorStraight*(1+ratio));
					rightMotor.setSpeed(motorStraight*(1-ratio));
					leftMotor.forward();
					rightMotor.forward();
				}
				if (difference < -5) //too close, move away faster
				{
					leftMotor.setSpeed(3*(motorStraight*(1+ratio)));
					rightMotor.setSpeed(motorStraight*(1-ratio)/2);
					leftMotor.forward();
					rightMotor.forward();
				}
				if(difference > 0) //far, move closer
				{
					leftMotor.setSpeed((motorStraight+50)*(1-ratio));
					rightMotor.setSpeed(2*motorStraight*(1+ratio)/3);
					leftMotor.forward();
					rightMotor.forward();
				}
			}
		}
	}

	
	public int readUSDistance() {
		return this.distance;
	}

}
