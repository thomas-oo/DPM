package Localization;

import java.util.ArrayList;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	public enum LocalizationType {FALLING_EDGE, RISING_EDGE};

	private Odometer odo;
	
	//fields for ultrasonic sensor
	private SampleProvider usSensor;	
	private float[] usData;
	private double usValue;
	
	private LocalizationType locType;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;


	private double maxDist = 200; //value to bring down all readings (returned by the ultrasonic sensor) to
	private int forwardSpeed = 100;
	
	//stores distance at which we detected alpha and beta in cm
	public double minimumDistA;
	public double minimumDistB;
	
	//stores angles of alpha and beta in radians
	public double alpha;
	public double beta;

	private double nowTheta;
	private double distance;

	private boolean faceWall; //flag to see if we are detecting a wall. also impacts calculation of deltaTheta
	
	//used for averaging theta at which we detected alpha
	private double sumOfMinTheta = 0;
	private double numberOfMinTheta = 0;

	//used for averaging theta at which we detected beta
	private double sumOfMinThetaB = 0;
	private double numberOfMinThetaB = 0;
	
	private double deltaTheta; //stores the adjustment we add to our current theta reading to finish localization
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
		if (locType == LocalizationType.FALLING_EDGE) //case falling edge
		{
			usValue = getFilteredData();
			State state = State.INITWALL;
			done = false;
			while(!done)
			{
				switch(state)
				{
				//INITWALL used for the robot to detect its initial position (is it facing a wall or not)
				case INITWALL:
					System.out.println("INITWALL");
					if (usValue >= 50) //when the robot isn't facing the wall
					{
						faceWall = false;
						state= State.FIRSTWALL;
						break;
					}
					else //if the robot starts off with facing the wall
					{
						faceWall = true;
						state = State.WALL;
						break;
					}
				case WALL: //makes you not face a wall
					System.out.println("WALL");
					while (usValue < 200)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					leftMotor.stop();
					rightMotor.stop();
					state = State.FIRSTWALL;
					break;

				case FIRSTWALL:
					//make robot rotate until it detects a distance of 50 from some wall
					while (usValue>=50) //the equal sign should also take care of overflow
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					
					System.out.println("Analyze start");
					
					//once the robot gets within a value of 50cm, have it start storing the distances it detects in order to find alpha
					//the goal of this case is to determine alpha
					usValue = getFilteredData(); //stores the most current distance
					minimumDistA = usValue; //stores the previous distance
					
					boolean firstWallDone = false; //flag
					
					//creating an arraylist to store the distances measured by the ultrasonic sensor
					ArrayList<double[]> storedData = new ArrayList<double[]>(); 
					while(!firstWallDone)
					{
						//while the robot detects that the current distance is smaller than the previous distance reported,
						//keep having it rotate until the current distance is larger (the robot has passed the minimum value between itself and
						//the wall)
						if(usValue <= minimumDistA) //collect data
						{
							minimumDistA = usValue;
							storedData.add(new double[] {odo.getTheta(), minimumDistA});

							rotateClockwise();
							usValue=getFilteredData();
						}
						else //analyzing data once the robot has detected that its previous distance is smaller than its current distance
						{
							leftMotor.stop();
							rightMotor.stop();
							//going through all the data aquired
							for(int i = storedData.size() - 1; i >= 0; i--)
							{
								//find out at which theta the robot has detected the minimum distance detected
								if((storedData.get(i))[1] == minimumDistA)
								{
									sumOfMinTheta += (storedData.get(i))[0]; //adding together all the thetas where minimum distance detected
									numberOfMinTheta ++; //the number of times the minimum distance has been reported
								}
							}
							//alpha will be equal to the average theta at which the minimal 
							//distances occured (sum of distances/number of times they occured)
							alpha = sumOfMinTheta / numberOfMinTheta;
							firstWallDone = true; //go out of the loop once alpha has been calculated
						}
					}

					//once it gets out of the loop, the robot should have overshoot, and it should determine the alpha
					//by using the midpoint of the array of sae values
					System.out.println("alpha found. Value of alpha:" + alpha*57.296);
					leftMotor.stop();
					rightMotor.stop();
					
					state = State.SECONDWALL;
					break;
					
					//this case is used to find the value of beta
				case SECONDWALL:
					
					int count = 0;
					while (count < 10) //to get out of wall, detect value of 200>= at least 5 times, this is used as a filter as the
						//ultrasonic sensor values isn't always reliable
					{
						while(usValue < 200)
						{
							rotateCounterClockwise(); //have the robot continue to rotate counter clockwise until it doesn't face a wall anymore
							usValue = getFilteredData();
						}
						rotateCounterClockwise();
						usValue = getFilteredData();
						count++;
					}
					
					//have the robot detect the second wall
					while (usValue>=50)
					{
						rotateCounterClockwise();
						usValue = getFilteredData();
					}
					
					System.out.println("Analyze start");
					//once the robot gets within a value of 50cm, have it start determining beta
					//and storing the distance recorded by the ultrasonic sensor.
					
					usValue = getFilteredData();
					minimumDistB = usValue; //initialize the minimumDist to the current distance
					boolean secondWallDone = false; //flag
					
					//Arraylist to which we store corresponding distance and theta readings
					ArrayList<double[]> storedDataB = new ArrayList<double[]>(); 
					
					while(!secondWallDone) //loops until we are done finding beta
					{	
						//as long as the most recent ultrasonic reading is lower than the previous one (falling edge)
						//keep storing new distance and theta readings into storedData
						if(usValue <= minimumDistB)
						{
							minimumDistB = usValue;
							storedDataB.add(new double[] {odo.getTheta(), minimumDistB});

							/*for (double[] p : storedDataB)
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);*/

							rotateCounterClockwise();
							usValue=getFilteredData();
							
							//this is used as a filter, override any faulty distance reported (when facing a wall, the value reported by the 
							//ultrasonic sensor shouldn't go beyond 200cm in the context of this lab
							if (usValue >=200)
							{
								usValue = minimumDistB;
							}
						}
						else //analyzing data to determine beta
						{
							boolean sumIsLarger = false;
							for(int i = 0; i < storedDataB.size(); i++)
							{
								if((storedDataB.get(i))[1] == minimumDistB)
								{
									//this is to handle the case where theta shoots from the 360 range to the 0 range, 
									//we want the values to stay constant so that we're able to detect the average theta accurately
									if((storedDataB.get(i))[0] < 0.25*Math.PI) //if the value of stored theta is smaller than 45degrees
									{
										sumOfMinThetaB += (storedDataB.get(i))[0] + 2*Math.PI; //converting everything into the 360 degrees range
										sumIsLarger = true;
									}
									else //if value of stored theta is larger than 45 degrees, add normally
									{
										sumOfMinThetaB += (storedDataB.get(i))[0];
									}
									numberOfMinThetaB ++;
								}
							}
							//the value of beta is the average value of theta at which the minimum distance occured
							beta = sumOfMinThetaB / numberOfMinThetaB;
							
							//this is not particularly needed
							if(sumIsLarger)
							{
								//beta -= 2*Math.PI;
							}
							secondWallDone = true; //goes here when the value of beta has been determined
						}
					}
					System.out.println("beta found. Value of beta"+beta*57.296);
					leftMotor.stop();
					rightMotor.stop();
					done = true;
					break;
				}
			}
			//have the robot turn to beta first in order to then make it turn to where the zero degree is supposed to be
			turnTo(beta);
			
			if (faceWall) //in this case, alpha>beta
			{
				//these calculations are taken from the tutorial
				deltaTheta = 0.25*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta+beta);
				System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (deltaTheta) + "turning to 0");
				turnTo(0);
			}
			else//in this case, beta>alpha
			{
				//these calculations are taken from the tutorial
				deltaTheta = 1.25*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta+beta);
				System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (deltaTheta) + "turning to 0");
				turnTo(0);
			}
			//override the values stored in the odometer
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		} 
		else if(locType == LocalizationType.RISING_EDGE)
		{
			usValue = getFilteredData(); //initialize usValue to what the ultrasonic sensor currently reads
			State state = State.INITWALL; //initialize the state
			done = false; //set to true once we detect beta (our strategy detects alpha and beta)

			while (!done) //finishes once beta is detected
			{
				switch (state) 
				{
				case INITWALL:
					System.out.println("INITWALL");
					//checks if we see a wall (we want to start alpha and beta only if we start facing a wall
					if (usValue >= 50) //not facing, rotate so that you are
					{
						faceWall = false;
						state = State.NOWALL;
						break;
					} 
					else //facing, ready to detect alpha and beta
					{
						faceWall = true;
						state = State.FIRSTWALL;
						break;
					}

				case NOWALL: //makes you face a wall
					
					System.out.println("NOWALL");
					
					//while values returned from ultrasonic sensor are more than 50 (we assume once we see 50, we see a wall)
					//rotate counterclockwise, this will make it so that the robot is facing the left wall to begin with
					while (usValue >= 50)
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
					
					System.out.println("FIRSTWALL");
					
					//get updated usValue, set flag to false and initialize an arraylist
					usValue = getFilteredData();
					boolean firstWallDone = false;
					
					//ArrayList storedData will store corresponding distance, and theta values returned from the ultrasonic sensor and odometer 
					ArrayList<double[]> storedData = new ArrayList<double[]>();
					
					//if we are still detecting a wall, keep rotating counter clockwise and storing corresponding distance and theta readings into storedData
					//once we stop detecting a wall (which will occur when we rotate counterclockwise past the back wall)
					//we have collected enough data to locate alpha
					while(!firstWallDone)
					{
						if(usValue < maxDist) //still detecting a wall
						{
							storedData.add(new double[] {odo.getTheta(), usValue});

							for (double[] p : storedData) //printer for debugging
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);

							rotateCounterClockwise();
							usValue = getFilteredData();
						}
						else //not detecting a wall (rising edge) thus ready to process data
						{
							leftMotor.stop();
							rightMotor.stop();
							
							//initialize the minimum distance we read
							minimumDistA = maxDist;
							
							//traverse the ArrayList from the MOST RECENT arrays to the EARLIEST arrays
							//traverse the ArrayList, looking at the distances (index 1) and store the minimum value until the next distance value we read in the ArrayList is larger
							//this avoids just finding the minimum distance in the entire Array:ist which may result in find b instead of finding a (which is unintended)
							
							//NOTE: arrayList add appends the latest double[] onto the END of the arrayList thus we will start at the very last index of the arrayList and go backwards
							
							//traverse array looking for the minimum distance
							for(int i = storedData.size() - 1; i >= 0; i--)
							{
								System.out.println("Distance values at index " + i + " : " + (storedData.get(i))[1]);
								System.out.println("Theta values at index " + i + " : " + (storedData.get(i))[0]);
								System.out.println("sumOfMinTheta: " + sumOfMinTheta);
								System.out.println("new minimumDistA: " + minimumDistA);
								
								
								//if we read multiple occurances of the minimum, we will compute a running sum of the corresponding theta values
								//and using the number of occurances of that minimum value, find an average theta which we will use for alpha
								//and additionally, whenever we see a new min dist, set the sum to be that min, and reset the number of occurances of the min back to 1
								
								if((storedData.get(i))[1] < minimumDistA)
								{
									minimumDistA = (storedData.get(i))[1];
									sumOfMinTheta = (storedData.get(i))[0];
									numberOfMinTheta = 1;
									System.out.println("new minDistA");
								}
								else if((storedData.get(i))[1] == minimumDistA)
								{
									sumOfMinTheta += (storedData.get(i))[0];
									numberOfMinTheta ++;
									System.out.println("repeated minDistA");
								}
								else
								{
									System.out.println("increase detected");
									break;
								}
							}
							alpha = sumOfMinTheta / numberOfMinTheta; //alpha is the average of the theta readings where we see a minimum
							System.out.println("sumOfMinTheta: " + sumOfMinTheta);
							System.out.println("numberOfMinTheta: " + numberOfMinTheta);
							firstWallDone = true; //we are done detecting alpha
						}
					}
					System.out.println("alpha: " + (alpha * 57.296));
					System.out.println("Completed first wall");
					turnTo(alpha);
					leftMotor.stop();
					rightMotor.stop();

					state = State.SECONDWALL; //ready to find beta now
					break;
				case SECONDWALL: //find beta
					
					System.out.println("SECONDWALL");
					//get updated usValue, set flag to false and initialize an arraylist
					usValue = getFilteredData();
					boolean secondWallDone = false;
					
					//ArrayList storedDataB will store corresponding distance, and theta values returned from the ultrasonic sensor and odometer
					ArrayList<double[]> storedDataB = new ArrayList<double[]>();
					
					while(usValue >= 50) //makes robot detect a wall again incase we overshot the wall (will be pointed at back wall once this is done)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					
					while(!secondWallDone)
					{
						if(usValue < maxDist) //still detecting a wall
						{
							storedDataB.add(new double[] {odo.getTheta(), usValue});
							for(double[] p : storedDataB)
							{
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);
							}
							rotateClockwise();
							usValue = getFilteredData();
						}
						else //not detecting a wall (rising edge) thus ready to process data
						{
							leftMotor.stop();
							rightMotor.stop();
							
							//initialize the minimum distance we read
							minimumDistB = maxDist;
							
							//traverse the ArrayList from the MOST RECENT arrays to the EARLIEST arrays
							//traverse the ArrayList, looking at the distances (index 1) and store the minimum value until the next distance value we read in the ArrayList is larger
							
							//traverse array looking for the minimum distance
							for(int i = storedDataB.size() - 1; i >= 0; i--)
							{
								System.out.println("Distance values at index " + i + " : " + (storedDataB.get(i))[1]);
								System.out.println("Theta values at index " + i + " : " + (storedDataB.get(i))[0]);
								System.out.println("sumOfMinTheta: " + sumOfMinThetaB);
								System.out.println("new minimumDistA: " + minimumDistB);
								
								//if we read multiple occurances of the minimum, we will compute a running sum of the corresponding theta values
								//and using the number of occurances of that minimum value, find an average theta which we will use for beta
								//and additionally, whenever we see a new min dist, set the sum to be that min, and reset the number of occurances of the min back to 1
								
								if((storedDataB.get(i))[1] < minimumDistB)
								{
									minimumDistB = (storedDataB.get(i))[1];
									sumOfMinThetaB = (storedDataB.get(i))[0];
									numberOfMinThetaB = 1;
									System.out.println("new minDistB");
								}
								else if((storedDataB.get(i))[1] == minimumDistB)
								{
									sumOfMinThetaB += (storedDataB.get(i))[0];
									numberOfMinThetaB ++;
									System.out.println("repeated minDistB");
								}
								else
								{
									System.out.println("increase detected");
									break;
								}
							}
							
							beta = sumOfMinThetaB / numberOfMinThetaB; //beta is the average of the theta readings where we see a minimum
							System.out.println("sumOfMinThetaB: " + sumOfMinThetaB);
							System.out.println("numberOfMinThetaB: " + numberOfMinThetaB);
							secondWallDone = true; //we are done detecting beta
						}
					}
					System.out.println("beta: " + (beta * 57.296));
					System.out.println("Completed second wall");
					leftMotor.stop();
					rightMotor.stop();
					
					done = true; //we have now found beta AND alpha, we are "done"
					break;
				}
			}
			
			turnTo(beta);
			
			if (faceWall) // A>B
			{
				deltaTheta = 0.25*Math.PI - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			else //A<B
			{
				deltaTheta = 1.25*Math.PI - ((alpha+beta)/2);
				odo.setTheta(deltaTheta + beta);
				turnTo(0);
			}
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		}
	}

	private void rotateCounterClockwise() 
	{
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.backward();
		rightMotor.forward();
	}

	private void rotateClockwise() 
	{
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.forward();
		rightMotor.backward();
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
		if(distance > maxDist)//so all distances bigger than 50 will be set to 50??
		{
			distance = maxDist;
		}
		else if(distance < 0)//smaller than 0 or 50?
		{
			distance = 50;
		}
		return distance;
	}

}


