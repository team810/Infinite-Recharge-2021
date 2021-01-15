// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Drivetrain m_drive = new Drivetrain();
  private final ShooterSystem m_shoot = new ShooterSystem();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  
  private JoystickButton intakeOut, intakeRun, feedRun, switchShoot, shootRun;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drive.setDefaultCommand(
      new RunCommand(()-> m_drive.tankDrive(left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeOut = new JoystickButton(left, 0);
      intakeOut.whenPressed(new InstantCommand(()->m_shoot.toggleSol(m_shoot.intakeSOL), m_shoot));
    switchShoot = new JoystickButton(left, 1);
      switchShoot.whenPressed(new InstantCommand(()->m_shoot.toggleSol(m_shoot.shooterSOL), m_shoot));
    shootRun = new JoystickButton(left, 2);
      shootRun.whileHeld(new RunCommand(()->m_shoot.shoot(1), m_shoot));
    intakeRun = new JoystickButton(left, 2);
      intakeRun.whileHeld(new RunCommand(()->m_shoot.runIntake(1), m_shoot));
    feedRun = new JoystickButton(left, 2);
      feedRun.whileHeld(new RunCommand(()->m_shoot.runFeed(1), m_shoot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
