package ca.mcgill.ecse211.WallFollowingLab;

import java.util.Arrays;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int filterDistance;
  private int previousError;
  private int[] lastFiveDistance = new int[5];
  private int index = 0;
  private int distError;

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

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
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
	else if(distError < 0)								//too close to wall - swerve right
	{
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + (bandCenter - Math.abs(this.distance)) * 5.5f); //constant higher because the distance closer to the wall is constrained (0-bandcenter)
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 2);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	else {									//too far from wall - swerve left
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 2);	
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + this.distance * 2.5f);	//smaller constant as distance can get much higher (bandcenter-255)
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