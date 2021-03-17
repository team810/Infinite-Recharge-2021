// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  
  private JoystickButton intakeOut, intakeRun, feedRun, feedRunAuto, switchShoot, shootRun, turnTarget, reset, resetE, servoTarget, servoBall;

  public HashMap<String, Trajectory> pathsTrajs = new HashMap<String, Trajectory>();
  public HashMap<String, Command> paths = new HashMap<String, Command>();

  public String[] trajNames = {"paths/GalacticBlueA.wpilib.json", "paths/GalacticBlueB.wpilib.json", "paths/GalacticRedA.wpilib.json", 
                                "paths/GalacticRedB.wpilib.json", "paths/BarrelRoll.wpilib.json", "paths/Slalom.wpilib.json",
                                "paths/Bounce1.wpilib.json", "paths/Bounce2.wpilib.json", "paths/Bounce3.wpilib.json", "paths/Bounce4.wpilib.json"};

  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(-left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    ); 
  }

  private void configureButtonBindings() {
    shootRun = new JoystickButton(left, 1);
      shootRun.whileHeld(new shoot(m_shoot, m_lime));
    intakeOut = new JoystickButton(left, 2);
      intakeOut.whenPressed(new InstantCommand(()->m_intake.toggleSol(m_intake.intakeSOL), m_intake));
    switchShoot = new JoystickButton(left, 3);
      switchShoot.whenPressed(new InstantCommand(()->m_shoot.toggleSol(m_shoot.shooterSOL), m_shoot));
    intakeRun = new JoystickButton(left, 4);
      intakeRun.whileHeld(new StartEndCommand(() -> m_intake.runIntake(-.5), ()-> m_intake.runIntake(0), m_intake));
    feedRun = new JoystickButton(right, 1);
      feedRun.whileHeld(new StartEndCommand(() -> m_feed.runFeed(1), ()-> m_feed.runFeed(0), m_feed));
    feedRunAuto = new JoystickButton(right, 3);
      feedRunAuto.whileHeld(new StartEndCommand(() -> m_feed.autoShoot(1), ()-> m_feed.autoShoot(0), m_feed));
    turnTarget = new JoystickButton(right, 2);
      turnTarget.whileHeld(new TurnToTarget(m_drive, m_lime));
    reset = new JoystickButton(right, 14);
      reset.whenPressed(new InstantCommand(()-> m_drive.navx.reset(), m_drive));
    resetE = new JoystickButton(right, 15);
      resetE.whenPressed(new InstantCommand(()-> m_drive.resetEncoders(), m_drive));
    servoTarget = new JoystickButton(left, 14);
      servoTarget.whenPressed(
        new SequentialCommandGroup(new InstantCommand(()->m_lime.m_servo.setAngle(100)), new InstantCommand(()->m_lime.pipeline.setNumber(0)), new InstantCommand(()->m_lime.setAngle(45)))
    );
    servoBall = new JoystickButton(left, 15);
      servoBall.whenPressed(
        new SequentialCommandGroup(new InstantCommand(()->m_lime.m_servo.setAngle(0)), new InstantCommand(()->m_lime.pipeline.setNumber(1)), new InstantCommand(()->m_lime.setAngle(0)))
    );
  }

  public Command getAutonomousCommand() {
    String path = "Bounce";
    // Reset odometry to starting pose of trajectory.
    m_drive.resetOdometry(pathsTrajs.get(path).getInitialPose());

    // Run path following command, then stop at the end.
    return paths.get(path).andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
