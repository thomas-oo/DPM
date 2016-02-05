package Navigation;
import lejos.hardware.motor.*;

public class BangBangController extends Thread{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int difference;
	
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		//Default Constructor
		this.bandCenter = bandCenter + 10;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow; //50
		this.motorHigh = motorHigh; //150
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorHigh);				// Start robot moving forward
		rightMotor.setSpeed(motorHigh);
	}
	
	public void processUSData(int distance) {
		this.distance = distance;
/*		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			// gives robot time to make a sharper turn
			filterControl ++;
		} 
		else if (distance >= 255){
			// true 255, therefore set distance to 255
			//also move closer slowly so you can detect a wall
			this.distance = 255;
			leftMotor.setSpeed(motorHigh - motorLow); //left motor moves slower
			rightMotor.setSpeed(motorHigh + motorLow); //right motor moves faster
			leftMotor.forward();
			rightMotor.forward();
			
		} 
		else if (distance < 255)
		{*/
			difference = distance - bandCenter;
			if (Math.abs(difference)<=(bandwidth)){
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorHigh);
				leftMotor.forward();
				rightMotor.forward();
			}
				
			
			//if EV3 is too far from the wall
			else if ((difference)>0){
				leftMotor.setSpeed(motorLow);
				rightMotor.setSpeed(motorHigh);
				leftMotor.forward();
				rightMotor.forward();
			}
			
			//if Ev3 is too close to the wall
			else if ((difference)<0){
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorLow);
				leftMotor.forward();
				rightMotor.forward();
			}
		}
		
		
		//go straight if less than bandwidth 
	public int readUSDistance() {
		return this.distance;
	}
}
