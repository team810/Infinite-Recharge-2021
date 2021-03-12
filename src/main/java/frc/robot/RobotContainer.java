// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Bounce;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSystem;

public class RobotContainer {
  public final Drivetrain m_drive = new Drivetrain();
  private final ShooterSystem m_shoot = new ShooterSystem();
  private final Feed m_feed = new Feed();
  private final Intake m_intake = new Intake();
  private final Limelight m_lime = new Limelight();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  
  private JoystickButton intakeOut, intakeRun, feedRun, switchShoot, shootRun, turnTarget, reset, resetE, servoTarget, servoBall;

  public RamseteCommand[] paths = new RamseteCommand[4];
  public Trajectory[] pathsTrajs = new Trajectory[4];
  public String[] trajNames = {"paths/GalacticBlueA.wpilib.json",
                          "paths/GalacticBlueB.wpilib.json", "paths/GalacticRedA.wpilib.json", "paths/GalacticRedB.wpilib.json"};
  public Bounce bounce;
  public RamseteCommand slalom, barrelRoll;

  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(-left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    ); 
  }

  private void configureButtonBindings() {
    shootRun = new JoystickButton(left, 1);
      shootRun.whileHeld(new shoot(m_shoot));
    intakeOut = new JoystickButton(left, 2);
      intakeOut.whenPressed(new InstantCommand(()->m_intake.toggleSol(m_intake.intakeSOL), m_intake));
    switchShoot = new JoystickButton(left, 3);
      switchShoot.whenPressed(new InstantCommand(()->m_shoot.toggleSol(m_shoot.shooterSOL), m_shoot));
    intakeRun = new JoystickButton(left, 4);
      intakeRun.whileHeld(new StartEndCommand(() -> m_intake.runIntake(-.5), ()-> m_intake.runIntake(0), m_intake));
    feedRun = new JoystickButton(right, 1);
      feedRun.whileHeld(new StartEndCommand(() -> m_feed.runFeed(1), ()-> m_feed.runFeed(0), m_feed));
    turnTarget = new JoystickButton(right, 2);
      turnTarget.whileHeld(new TurnToTarget(m_drive, m_lime));
    reset = new JoystickButton(right, 14);
      reset.whenPressed(new InstantCommand(()-> m_drive.navx.reset(), m_drive));
    resetE = new JoystickButton(right, 15);
      resetE.whenPressed(new InstantCommand(()-> m_drive.resetEncoders(), m_drive));
    servoTarget = new JoystickButton(left, 14);
      servoTarget.whenPressed(
        new SequentialCommandGroup(new InstantCommand(()->m_lime.m_servo.setAngle(180)), new InstantCommand(()->m_lime.pipeline.setNumber(0)))
      );
    servoBall = new JoystickButton(left, 15);
      servoBall.whenPressed(
        new SequentialCommandGroup(new InstantCommand(()->m_lime.m_servo.setAngle(-30)), new InstantCommand(()->m_lime.pipeline.setNumber(1)))
      );
  }

  public Command getAutonomousCommand() {
    //FOR AUTONAV CHALLENGE
    //return slalom.andThen(() -> m_drive.tankDrive(0, 0));

    //FOR GALACTIC CHALLENGE
    int path = m_lime.determinePath();
    m_drive.resetOdometry(pathsTrajs[path].getInitialPose());
    return paths[path].deadlineWith(
      new InstantCommand(()->m_intake.toggleSol(m_intake.intakeSOL)),
      new RunCommand(()->m_intake.runIntake(-1), m_intake)
    );
  }
}
