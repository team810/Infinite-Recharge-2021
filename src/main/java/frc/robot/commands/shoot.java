// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSystem;

public class shoot extends CommandBase {
  /** Creates a new shoot. */
  ShooterSystem m_shoot;
  CANPIDController m_pidController;
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM,
    maxVel, maxAcc;
  int smartMotionSlot;
  ShuffleboardTab tab;
  private BooleanSupplier cShoot = ()->false;
  private NetworkTableEntry setSpeed, setP, setI, setD, setF, speed, canShoot;

  public shoot(ShooterSystem m_shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shoot = m_shoot;
    m_pidController = m_shoot.getShooter().getPIDController();
    addRequirements(m_shoot);
    smartdashboardInit();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kIz = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 5600; // rpm
    maxAcc = 2000;

    // set PID coefficients
    smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(5, smartMotionSlot);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kP = setP.getDouble(0); 
    kI = setI.getDouble(0);
    kD = setD.getDouble(0); 
    kFF = setF.getDouble(0);
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    double setPoint, processVariable;

    setPoint = setSpeed.getDouble(5000);
    m_pidController.setReference(setPoint, ControlType.kVelocity);
    processVariable = m_shoot.getShooter().getEncoder().getVelocity();
    //SmartDashboard.putNumber("Shooter Speed", processVariable);
    speed.setDouble(processVariable);
    if(processVariable < (setPoint + 100) && processVariable > (setPoint - 100)){
      cShoot = ()-> true;
    }else{
      cShoot = ()-> false;
    }
    canShoot.setBoolean(cShoot.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoot.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void smartdashboardInit(){
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter System");
    Shuffleboard.selectTab("Shooter System");
    setSpeed = tab.add("Set Speed", 5000).getEntry();
    speed = tab.add("Speed", 0).getEntry();
    setP = tab.addPersistent("P", Constants.kPShooter).getEntry();
    setI = tab.addPersistent("I", Constants.kIShooter).getEntry();
    setD = tab.addPersistent("D", Constants.kDShooter).getEntry();
    setF = tab.addPersistent("F", Constants.kFShooter).getEntry();
    canShoot = tab.add("Shoot?", cShoot.getAsBoolean()).getEntry();
  }
}
