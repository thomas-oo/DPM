package Localization;

import java.util.ArrayList;
import java.util.Arrays;

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
	private int forwardSpeed = 70;
	
	//stores distance at which we detected alpha and beta in cm
	public double minimumDistA;
	public double minimumDistB;
	
	//stores angles of alpha and beta in radians
	public double alpha;
	public double beta;

	private double nowTheta;
	private double distance;

	private boolean faceWall; //flag to see if we are detecting a wall
	
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
						System.out.println("WALL LOOP");
						rotateClockwise();
						usValue = getFilteredData();
					}
					leftMotor.stop();
					rightMotor.stop();
					state = State.FIRSTWALL;
					break;

				case FIRSTWALL:
					//make robot rotate until it detects a distance of 50 
					while (usValue>=50) //the equal sign should also take care of overflow
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					System.out.println("Analyze start");
					//once the robot gets within a value of 50cm, have it start determining alpha
					//goal: to detect alpha
					usValue = getFilteredData(); 
					minimumDistA = usValue;
					boolean firstWallDone = false;
					ArrayList<double[]> storedData = new ArrayList<double[]>(); 
					while(!firstWallDone)
					{
						System.out.println("outLoop usValue" + usValue);
						System.out.println("outLoop inDistA" + minimumDistA);
						System.out.println();
						System.out.println();
						if(usValue <= minimumDistA) //collecting data
						{
							System.out.println("insideLoop usValue" + usValue);
							System.out.println("insideLoop inDistA" + minimumDistA);
							minimumDistA = usValue;
							storedData.add(new double[] {odo.getTheta(), minimumDistA});

							for (double[] p : storedData)
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);

							rotateClockwise();
							usValue=getFilteredData();
						}
						else //analyzing data
						{
							leftMotor.stop();
							rightMotor.stop();
							for(int i = storedData.size() - 1; i >= 0; i--)
							{
								if((storedData.get(i))[1] == minimumDistA)
								{
									sumOfMinTheta += (storedData.get(i))[0];
									System.out.println("sumOfMinTheta" + sumOfMinTheta);
									numberOfMinTheta ++;
									System.out.println("numberOfMinTheta" + numberOfMinTheta);
								}
							}
							alpha = sumOfMinTheta / numberOfMinTheta;
							firstWallDone = true;
						}
					}

					//once it gets out of the loop, the robot should have overshoot, and it should determine the alpha
					//by using the midpoint of the array of sae values
					System.out.println(alpha * 57.296);
					System.out.println("Complete");
					leftMotor.stop();
					rightMotor.stop();
					
					state = State.SECONDWALL;
					break;
				case SECONDWALL:
					int count = 0;
					while (count < 5) //to get out of wall, detect value of 200>= , 5 times
					{
						while(usValue < 200)
						{
							rotateCounterClockwise();
							usValue = getFilteredData();
						}
						rotateCounterClockwise();
						usValue = getFilteredData();
						count++;
					}
					
					while (usValue>=50) //to turn to left wall
					{
						rotateCounterClockwise();
						usValue = getFilteredData();
					}
					
					System.out.println("Analyze start");
					//once the robot gets within a value of 50cm, have it start determining beta
					//goal: to detect beta
					
					usValue = getFilteredData();
					minimumDistB = usValue; //initialize the minimumDist to the current distance
					boolean secondWallDone = false; //flag
					
					//Arraylist to which we store corresponding distance and theta readings
					ArrayList<double[]> storedDataB = new ArrayList<double[]>(); 
					
					while(!secondWallDone) //loops until we are done finding beta
					{
						//printing for debugging
						System.out.println("outLoop usValue" + usValue);
						System.out.println("outLoop inDistB" + minimumDistB);
						System.out.println();
						System.out.println();
						
						//as long as the most recent ultrasonic reading is lower than the previous one (falling edge)
						//keep storing new distance and theta readings into storedData
						if(usValue <= minimumDistB)
						{
							System.out.println("insideLoop usValue" + usValue);
							System.out.println("insideLoop inDistB" + minimumDistB);
							minimumDistB = usValue;
							storedDataB.add(new double[] {odo.getTheta(), minimumDistB});

							for (double[] p : storedDataB)
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);

							rotateCounterClockwise();
							usValue=getFilteredData();
							if (usValue ==200)
							{
								usValue = minimumDistB;
							}
						}
						else //analyzing data
						{
							boolean flag = false;
							boolean sumIsLarger = false;
							for(int i = 0; i < storedDataB.size(); i++)
							{
								if((storedDataB.get(i))[1] == minimumDistB)
								{
									if((storedDataB.get(i))[0] < 0.25*Math.PI)
									{
										sumOfMinThetaB += (storedDataB.get(i))[0] + 2*Math.PI;
										System.out.println("sumOfMinThetaB: " + sumOfMinThetaB);
										sumIsLarger = true;
									}
									else
									{
										sumOfMinThetaB += (storedDataB.get(i))[0];
										System.out.println("sumOfMinThetaB: " + sumOfMinThetaB);
									}
									numberOfMinThetaB ++;
									System.out.println("numberOfMinThetaB: " + numberOfMinThetaB);
								}
							}
							beta = sumOfMinThetaB / numberOfMinThetaB;
							if(sumIsLarger)
							{
								//beta -= 2*Math.PI;
							}
							secondWallDone = true;
						}
					}

					//once it gets out of the loop, the robot should have overshoot, and it should determine the alpha
					//by using the midpoint of the array of sae values
					System.out.println(beta * 57.296);
					System.out.println("Complete");
					leftMotor.stop();
					rightMotor.stop();
					done = true;
					break;
				}
			}
			turnTo(beta);
			if (faceWall) //A>B
			{
				deltaTheta = 0.25*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta+beta);
				System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (deltaTheta) + "turning to 0");
				turnTo(0);
			}
			else//A<B
			{
				deltaTheta = 1.25*Math.PI -((alpha+beta)/2);
				odo.setTheta(deltaTheta+beta);
				System.out.println("faceWall: " + faceWall + " Setting theta to: "+ (deltaTheta) + "turning to 0");
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
					System.out.println("INITWALL");
					if (usValue >= 50) 
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

				case NOWALL: //makes you facce a wall
					System.out.println("NOWALL");
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
					
					usValue = getFilteredData();
					boolean firstWallDone = false;
					ArrayList<double[]> storedData = new ArrayList<double[]>();
					
					while(!firstWallDone)
					{
						if(usValue < maxDist) //if usValue is less than 200, (still detecting a wall)
						{
							storedData.add(new double[] {odo.getTheta(), usValue}); //store that distance into an arraylist with its corresponding theta

							for (double[] p : storedData) //printer for debugging
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);

							rotateCounterClockwise(); //rotate clockwise
							usValue = getFilteredData(); //get an updated usValue
						}
						else //thus now ready to process distance.
						{
							leftMotor.stop();
							rightMotor.stop();
							
							//traverse the ArrayList from the MOST RECENT arrays to the EARLIEST arrays
							//traverse the arraylist, looking at the distances ([1]) and store the min value until the "next" distance value is larger
							//this avoids just finding the min distance in the entire arraylist which may make it possible to find b instead of finding a (which is unintended)
							
							//NOTE: arrayList add appends the latest double[] onto the END of the arrayList
							
							minimumDistA = maxDist;
							
							for(int i = storedData.size() - 1; i >= 0; i--)
							{
								System.out.println("Distance values at index " + i + " : " + (storedData.get(i))[1]);
								System.out.println("Theta values at index " + i + " : " + (storedData.get(i))[0]);
								System.out.println("sumOfMinTheta: " + sumOfMinTheta);
								System.out.println("new minimumDistA: " + minimumDistA);
								
								if((storedData.get(i))[1] < minimumDistA) //whenever we see a new min dist, set the sum to be that min
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
							alpha = sumOfMinTheta / numberOfMinTheta;
							System.out.println("sumOfMinTheta: " + sumOfMinTheta);
							System.out.println("numberOfMinTheta: " + numberOfMinTheta);
							firstWallDone = true;
						}
					}
					System.out.println("alpha: " + (alpha * 57.296));
					System.out.println("Completed first wall");
					leftMotor.stop();
					rightMotor.stop();

					state = State.SECONDWALL; //ready to find beta now
					break;
				case SECONDWALL: //find beta
					System.out.println("SECONDWALL");
					
					usValue = getFilteredData();
					boolean secondWallDone = false;
					ArrayList<double[]> storedDataB = new ArrayList<double[]>();
					while(usValue >= 50) //gets it to detected a wall again (will be pointed at back wall once this is done)
					{
						rotateClockwise();
						usValue = getFilteredData();
					}
					
					
					while(!secondWallDone)
					{
						if(usValue < maxDist)
						{
							storedDataB.add(new double[] {odo.getTheta(), usValue});
							for(double[] p : storedDataB)
							{
								System.out.println("theta : " + p[0] + ", distance: " + p[1]);
							}
							rotateClockwise();
							usValue = getFilteredData();
						}
						else
						{
							leftMotor.stop();
							rightMotor.stop();
							
							minimumDistB = maxDist;
							for(int i = storedDataB.size() - 1; i >= 0; i--)
							{
								System.out.println("Distance values at index " + i + " : " + (storedDataB.get(i))[1]);
								System.out.println("Theta values at index " + i + " : " + (storedDataB.get(i))[0]);
								System.out.println("sumOfMinTheta: " + sumOfMinThetaB);
								System.out.println("new minimumDistA: " + minimumDistB);
								
								if((storedDataB.get(i))[1] < minimumDistB) //whenever we see a new min dist, set the sum to be that min
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
							
							beta = sumOfMinThetaB / numberOfMinThetaB;
							System.out.println("sumOfMinThetaB: " + sumOfMinThetaB);
							System.out.println("numberOfMinThetaB: " + numberOfMinThetaB);
							secondWallDone = true;
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
			//we get to this part of the code once done = true;
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
	private double convertToDeg(double theta) 
	{
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


