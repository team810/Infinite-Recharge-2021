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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intake;
  public DoubleSolenoid intakeSOL = new DoubleSolenoid(Constants.INTAKE_FORWARD, Constants.INTAKE_REVERSE);

  public Intake() {
    intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void runIntake(double speed){
    intake.set(speed);
  }

  public void toggleSol(DoubleSolenoid d){
    if(d.get() == Value.kForward){
      d.set(Value.kReverse);
    }else{
      d.set(Value.kForward);
    }
  }

  public void toggleSol(){
    if(intakeSOL.get() == Value.kForward){
      intakeSOL.set(Value.kReverse);
    }else{
      intakeSOL.set(Value.kForward);
    }
  }
}
