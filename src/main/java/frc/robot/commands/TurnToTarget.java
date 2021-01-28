// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends PIDCommand {
  /** Creates a new TurnToTarget. */
  public TurnToTarget(Drivetrain m_drive, Limelight m_lime) {
    super(
        // The controller that the command will use
        new PIDController(.1, 0, 0.01),
        // This should return the measurement
        () -> m_drive.navx.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> Constants.tx.getDouble(0.0),
        // This uses the output
        output -> {
          m_drive.arcadeDrive(0, output);
        });
    addRequirements(m_drive);
    getController().setTolerance(.5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
