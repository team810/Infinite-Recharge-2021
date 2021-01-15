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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public final CANSparkMax front_L = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
  public final CANSparkMax front_R = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);
  public final CANSparkMax back_L =  new CANSparkMax(Constants.BACKL, MotorType.kBrushless);
  public final CANSparkMax back_R = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);

    
  private final DifferentialDrive drive = new DifferentialDrive(back_L, front_R);

  public final AHRS navx = new AHRS(SPI.Port.kMXP); // change to I2C if not working
  
  private DifferentialDriveOdometry m_odometry =
    new DifferentialDriveOdometry(navx.getRotation2d());
  
  private DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);

  public Drivetrain() {
    drive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }


}
