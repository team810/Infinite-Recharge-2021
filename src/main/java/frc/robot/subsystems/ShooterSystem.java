// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSystem extends SubsystemBase {
  /** Creates a new ShooterSystem. */
  private CANSparkMax intake, feed, shoot;
  public DoubleSolenoid intakeSOL = new DoubleSolenoid(Constants.INTAKE_FORWARD, Constants.INTAKE_REVERSE);
  public DoubleSolenoid shooterSOL = new DoubleSolenoid(Constants.SHOOTER_FORWARD, Constants.SHOOTER_REVERSE);

  public ShooterSystem() {
    intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
    feed = new CANSparkMax(Constants.FEED_MOTOR, MotorType.kBrushless);
    shoot = new CANSparkMax(Constants.SHOOT_MOTOR, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speed){
    shoot.set(speed);
  }

  public void runIntake(double speed){
    intake.set(speed);
  }

  public void runFeed(double speed){
    feed.set(speed);
  }

  public void toggleSol(DoubleSolenoid d){
    if(d.get() == Value.kForward){
      d.set(Value.kReverse);
    }else{
      d.set(Value.kForward);
    }
  }
}
