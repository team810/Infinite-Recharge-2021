// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToTarget extends CommandBase {
  private Drivetrain m_drive;
  private Limelight m_lime;
  private double kP, kI, kD, minThreshold, error;
  ShuffleboardTab tab;
  private NetworkTableEntry setMin, setP, setI, setD, angle, str;
  private double steer = 0;

  /** Creates a new TurnToTarget. */
  public TurnToTarget(Drivetrain m_drive, Limelight m_lime) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(m_drive);
    addRequirements(m_lime);
    this.m_drive = m_drive;
    this.m_lime = m_lime;
    shuffleboardInit();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kP = setP.getDouble(-.1);
    kI = setI.getDouble(0);
    kD = setD.getDouble(0);
    minThreshold = setMin.getDouble(0.05);
    error = angle.getDouble(0);
    steer = 0;

    if(Constants.tx.getDouble(0) > 1.0){
      steer = (kP * error) - minThreshold; 
    }else if(Constants.tx.getDouble(0) < 1.0){
      steer = (kP * error) + minThreshold;
    }
    str.setNumber(steer);
    m_drive.arcadeDrive(0, steer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return steer < Math.abs(minThreshold);
  }

  private void shuffleboardInit(){
    tab = Shuffleboard.getTab("Limelight");
    setMin = tab.add("Min Threshold", 0.05).getEntry();
    setP = tab.addPersistent("P", 6e-6).getEntry();
    setI = tab.addPersistent("I", 4e-7).getEntry();
    setD = tab.addPersistent("D", 7e-6).getEntry();
    angle = tab.add("Curr Angle", m_drive.navx.getAngle()).getEntry();
    str = tab.add("Curr Steer", steer).getEntry();
  }
}
