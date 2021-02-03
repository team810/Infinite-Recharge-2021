// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private NetworkTableEntry setSpeed, setP, setI, setD, speed, canShoot;

  public shoot(ShooterSystem m_shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shoot = m_shoot;
    m_pidController = m_shoot.getShooter().getPIDController();
    addRequirements(m_shoot);
    //SmartDashboard.putNumber("SetSpeed", 3000);
    //
    
    smartdashboardInit();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = 6e-6; 
    kI = 4e-7;
    kD = 7e-6; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 5700; // rpm
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
    kP = setP.getDouble(6e-6); 
    kI = setI.getDouble(4e-7);
    kD = setD.getDouble(7e-6); 

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
    setSpeed = tab.add("Speed", 0).getEntry();
    setP = tab.addPersistent("P", 6e-6).getEntry();
    setI = tab.addPersistent("I", 4e-7).getEntry();
    setD = tab.addPersistent("D", 7e-6).getEntry();
    canShoot = tab.add("Shoot?", cShoot.getAsBoolean()).getEntry();
  }
}
