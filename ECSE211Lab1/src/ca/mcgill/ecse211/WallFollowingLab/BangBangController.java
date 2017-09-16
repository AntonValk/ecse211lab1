package ca.mcgill.ecse211.WallFollowingLab;

import lejos.hardware.motor.*;
import java.util.Arrays;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int distError;
  private int[] lastFiveDistance = new int[5];
  private int index = 0;
  
  private int filterControl;
  private static final int FILTER_OUT = 20;


  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    Arrays.fill(lastFiveDistance, 30);
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    filterControl = 0;
    distError = 0;
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
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    distError = this.distance - this.bandCenter;
    if ((distance >= 70 || distance <=0) && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
        return;
      } else if (distance >= 70) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        this.distance = 70;
      } else if (distance <= 70){
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = distance;
      }
    
    if (Math.abs(distError) <= this.bandwidth) { 
    	// Within limits, same speed 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start moving forward 
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    else if (distError > 0) {
    	WallFollowingLab.leftMotor.setSpeed(motorLow-30);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+30); 
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    else if (distError < 0) {
       	WallFollowingLab.leftMotor.setSpeed(motorHigh+37); 
    	WallFollowingLab.rightMotor.setSpeed(motorLow-55);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int readFilterControl(){
	  return this.distError;
  }
}