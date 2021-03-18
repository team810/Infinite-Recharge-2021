// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnDegreesPID extends PIDCommand {
  /** Creates a new TurnDegreesPID. */
  public TurnDegreesPID(Drivetrain m_drive, double degrees) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> m_drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> Math.IEEEremainder(m_drive.getHeading() + degrees, 360),
        // This uses the output
        output -> {
          m_drive.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
