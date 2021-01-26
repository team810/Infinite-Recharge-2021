// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private double resetR = 0;
  private double resetL = 0;
  public final CANSparkMax front_L = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
  public final CANSparkMax front_R = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
  public final CANSparkMax back_L =  new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
  public final CANSparkMax back_R = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    
  private final DifferentialDrive drive = new DifferentialDrive(back_L, front_R);

  public final AHRS navx = new AHRS(SPI.Port.kMXP); // change to I2C if not working
  
  private DifferentialDriveOdometry m_odometry =
    new DifferentialDriveOdometry(navx.getRotation2d());
  
  public DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);

  public DifferentialDrivetrainSim m_drivetrainSim;

  //public EncoderSim m_leftEncoderSim = front_L.getEncoder();

  public Drivetrain() {
    drive.setSafetyEnabled(false);
    back_L.restoreFactoryDefaults();
    back_R.restoreFactoryDefaults();
    front_R.restoreFactoryDefaults();
    front_L.restoreFactoryDefaults();
    
    front_L.follow(back_L);
    back_R.follow(front_R);

    m_drivetrainSim = new DifferentialDrivetrainSim(
      Constants.kDrivetrainPlant, DCMotor.getNEO(2), 12.75, 
      Constants.TRACK_WIDTH_METERS, Constants.RADIUS, 
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    //set conversion factors
    front_L.getEncoder().setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE));
    front_R.getEncoder().setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE));

    front_L.getEncoder().setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE));
    front_R.getEncoder().setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      navx.getRotation2d(), getLeftEncoderPos(),
      getRightEncoderPos()
    );
    SmartDashboard.putNumber("Velocity", front_L.getEncoder().getVelocity());
    SmartDashboard.putNumber("Heading", navx.getRotation2d().getDegrees());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  public void resetEncoders(){
    resetL = front_L.getEncoder().getPosition();
    resetR = front_R.getEncoder().getPosition();
  }

  public double getRightEncoderPos(){
    return front_R.getEncoder().getPosition() - resetR;
  }
  public double getLeftEncoderPos(){
    return front_L.getEncoder().getPosition() - resetL;
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    front_R.setVoltage(rightVolts);
    front_L.setVoltage(leftVolts);
    drive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(front_L.getEncoder().getVelocity(), front_R.getEncoder().getVelocity());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void arcadeDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public double getDrawnCurrentAmps(){ return m_drivetrainSim.getCurrentDrawAmps();}
}
