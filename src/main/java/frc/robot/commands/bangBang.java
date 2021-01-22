// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;

public class bangBang extends CommandBase {
  /** Creates a new bangBang. */
  private ShooterSystem m_shoot;
  private double setpoint;
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  public bangBang(ShooterSystem m_shoot, double setpoint) {
    this.m_shoot = m_shoot;
    this.setpoint = tab.add("Speed Setpoint (B.B)", 1).getEntry().getDouble(500);
    addRequirements(m_shoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shoot.getShooter().getEncoder().getVelocity() < setpoint){
      m_shoot.shoot(1);
    }else{
      m_shoot.shoot(0);
    }
    //Shuffleboard.getTab("Shooter").add("Velocity (RPM)", m_shoot.getShooter().getEncoder().getVelocity());
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
