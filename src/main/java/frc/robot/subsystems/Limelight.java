/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Configures limelight settings
 */
public class Limelight extends SubsystemBase {

  NetworkTableEntry ledMode = Constants.ledMode;
  NetworkTableEntry camMode = Constants.camMode;
  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  

  
 

  //Servo mover = new Servo(Constants.SERVO_MOTOR);
  
  double validTarget = Constants.tv.getDouble(0.0);

  public Servo m_servo = new Servo(2);

  private final AnalogInput ultrasonic = new AnalogInput(2); 

  private final int OFFSET = 5;

  //public Ultrasonic ultrasonic = new Ultrasonic(1);
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance (Ultrasonic)", getDistanceUltrasonic());
    SmartDashboard.putNumber("Distance (Limelight)", getDistance());
    SmartDashboard.putNumber("TY", Constants.ty.getDouble(0.0));

    getDistance();

    //System.out.println(m_servo.getAngle());

    //if(xPos.length > 1)
    //System.out.println(xPos[1] - xPos[0]);
    //changeAngle(.5);
  }

  /**
   * Sets limelight to vision processing mode and sets vision pipeline to 0
   */
  public void visionMode() {
    camMode.setNumber(0); // sets camera to vision processing mode
    pipeline.setNumber(0);
  }

  public void driverMode() {
    camMode.setNumber(1); // sets camera to driving mode
    pipeline.setNumber(0);
  }

  

  /**
   * Forces on light
   */
  public void lightOn() {
    ledMode.setNumber(3);
  }

  /**
   * Forces off light
   */
  public void lightOff() {
    ledMode.setNumber(1);
  }

  /**
   * Changes light settings according to how vision pipeline is set
   */
  public void lightAuto() {
    ledMode.setNumber(0);
  }
    
  /**
   * Checks if there is no valid targets, which is then sent to isFinsished() 
   */
  public boolean noValidTarget() {
    if (validTarget == 0) {
      SmartDashboard.putBoolean("Target", false);
      return true; 

    } else {
      SmartDashboard.putBoolean("Target", true);
      return false;
      
    }
  }

  public boolean isValidTarget(){
    if(Constants.tv.getDouble(0.0) == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public double getRPM(){
    return 0;
  }

  public double getAngle(){
    return Constants.ty.getDouble(0.0);
  }

  public double getDistanceUltrasonic(){
    /*
      Ultrasonic.getValue() returns voltage.
      Multiplying with .125 returns in CENTIMETERS
      Converting from cm to inches is just dividing by 2.54 
    */
    return (ultrasonic.getValue() * 0.125) / 2.54;
  }

  public double getDistance(){
    double angle = m_servo.getAngle() + Constants.ty.getDouble(0.0) - OFFSET; 
    double distToTarget = (98.75 - 19)  / Math.tan(Math.toRadians(angle));
    return distToTarget;
  }
  /*
    PATH IDS: 
    0: "paths/GalacticBlueA.wpilib.json",
    1: "paths/GalacticBlueB.wpilib.json", 
    2: "paths/GalacticRedA.wpilib.json", 
    3: "paths/GalacticRedB.wpilib.json"
  */
  public int determinePath(){
   /* NetworkTable ballTable = NetworkTableInstance.getDefault().getTable("Balls");
    NetworkTableEntry positions = ballTable.getEntry("x");
    
    double[] xPos = positions.getDoubleArray(new double[0]); 

    if(xPos[1] > xPos[0]){
      return 2;
    }else{
      return 0;
    }
    */
    return 2;
  
  }


  public void setServoDefault(){
    m_servo.setAngle(0);
  }
  
}