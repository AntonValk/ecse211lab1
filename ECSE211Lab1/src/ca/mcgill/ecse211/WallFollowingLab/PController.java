import java.util.Arrays;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 195; // Default speed for the motor  (deg/sec)
  private static final int FILTER_OUT = 20;	// Maximum number of times we filter large inputs

  private final int bandCenter; // Desired offset from the wall (cm)
  private final int bandWidth;  // Width of dead band (cm)
  private int distance; // Distance from the wall (cm)
  private int filterControl; // Counts the amount of times we have filtered the input
  private int filterDistance; // Sets a maximum value for the distance variable
  private int previousError; // The previous offset from the band center (cm)
  private int[] lastFiveDistance = new int[5]; // Array of the last 5 US outputs
  private int index = 0; //Index to keep track of the array part we are accessing.
  private int distError; // Current offset from the band center (cm)

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;
    this.filterDistance = 0;
    this.previousError = 0;
    Arrays.fill(lastFiveDistance, 30);
    this.distError = 0;
    
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  
	  // Because the US data tends to be inconsistent we take the average of 
	  // 5 values from the sensor and set it equal to distance.
	  // To prevent high sensor values from skewing the average we set values greater than 100 to 100.
	  
	  distance = Math.min(distance, 100);
	    this.index++;
	    if(this.index == 5) {
	    	this.index = 0;
	    }
	    
	    lastFiveDistance[index] = distance;
	    
	    int average = 0;
	    for (int i=0; i<lastFiveDistance.length; i++) {
	    	average+=lastFiveDistance[i];
	    }
	    average = average / 5;
	    
	    this.distance = average;

    // rudimentary filter - toss out invalid samples above a certain value
	    
	    distError = this.distance - this.bandCenter;	//offset between current position and ideal distance from wall (in cm)
	   if ((distance >= 70 || distance <=0) && filterControl < FILTER_OUT) {
	       // bad value, do not set the distance var, however do increment the
	       // filter value
	        filterControl++;
	        return;
	      } else if (distance >= 70) {
	        // We have repeated large values, so there must actually be nothing
	        // there: leave the distance alone
	        this.distance = distance;
	      } else if (distance <= 70){
	        // distance went below 255: reset filter and leave
	        // distance alone.
	        filterControl = 0;
	        this.distance = distance;
	      }

    // TODO: process a movement based on the us distance passed in (P style)
   
	//check if the reported error suddenly changed - ignore value if so (false positive)
	if(distError - this.previousError > this.filterDistance)
	{
		this.previousError = distError;
		return;
	}
	if(Math.abs(distError) <= this.bandWidth)	//straight (in dead band)
	{
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	else if(distError < 0)								//too close to wall - we must turn right
	{
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + (bandCenter - Math.abs(this.distance)) * 5.0f); //constant higher (if we don't turn fast we will hit the wall!)
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 2);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	else {									//too far from wall - we must turn left
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 2);	
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + this.distance * 2.5f);	//smaller constant as distance grows faster on left turns
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	this.previousError = distError;	//record last error
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int readFilterControl(){
	  return filterControl;
  }

}
