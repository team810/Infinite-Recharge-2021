// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ShooterSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BaseShoot extends ParallelCommandGroup {
  /** Creates a new BaseShoot. */
  public BaseShoot(ShooterSystem m_shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> m_shoot.shoot(1), m_shoot).withTimeout(2),
      new RunCommand(() -> m_shoot.runFeed(1), m_shoot).withTimeout(2)
    );
  }
}
