// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class shoot extends CommandBase {
  /** Creates a new shoot. */
  ShooterSystem m_shoot;
  CANPIDController m_pidController;
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM,
    maxVel, maxAcc;
  int smartMotionSlot;
  ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  public shoot(ShooterSystem m_shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shoot = m_shoot;
    m_shoot.getShooter().getPIDController();
    addRequirements(m_shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5600;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(10, smartMotionSlot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read PID coefficients from SmartDashboard
    double p = tab.add("P Gain", kP).getEntry().getDouble(0);
    double i = tab.add("I Gain", kI).getEntry().getDouble(0);
    double d = tab.add("D Gain", kD).getEntry().getDouble(0);
    double iz = tab.add("I Zone", kIz).getEntry().getDouble(0);
    double ff = tab.add("Feed Forward", kFF).getEntry().getDouble(0);
    double max = tab.add("Max Output", kMaxOutput).getEntry().getDouble(0);
    double min = tab.add("Min Output", kMinOutput).getEntry().getDouble(0);
    double maxV = tab.add("Max Velocity", maxVel).getEntry().getDouble(0);
    double minV = tab.add("Min Velocity", maxVel * -1).getEntry().getDouble(0);
    double maxA = tab.add("Max Acceleration", maxAcc).getEntry().getDouble(0);
    double allE = tab.add("Allowed Closed Loop Error", 10).getEntry().getDouble(0);
    
    double maxE = tab.add("Max Closed Loop Error", 10).getEntry().getDouble(0);;

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != maxVel * -1)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); maxVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != maxE)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); maxE = allE; }

    double setPoint, processVariable;
    boolean mode = tab.add("Mode", 0).getEntry().getBoolean(false);
    if(mode) {
      setPoint = tab.add("Set Velocity", 0).getEntry().getDouble(0);
      m_pidController.setReference(setPoint, ControlType.kVelocity);
      processVariable = m_shoot.getShooter().getEncoder().getVelocity();
    } else {
      setPoint = tab.add("Set Position", 0).getEntry().getDouble(0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = m_shoot.getShooter().getEncoder().getPosition();
    }
    
    tab.add("SetPoint", setPoint).getEntry().getDouble(setPoint);
    tab.add("Process Variable", processVariable).getEntry().getDouble(processVariable);
    tab.add("Output", m_shoot.getShooter().getAppliedOutput()).getEntry().getDouble(m_shoot.getShooter().getAppliedOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
