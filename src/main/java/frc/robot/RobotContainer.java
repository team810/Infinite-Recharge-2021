// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Drivetrain m_drive = new Drivetrain();
  private final ShooterSystem m_shoot = new ShooterSystem();
  private final Feed m_feed = new Feed();
  private final Intake m_intake = new Intake();
  private final Limelight m_lime = new Limelight();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  
  private JoystickButton intakeOut, intakeRun, feedRun, switchShoot, shootRun, turnTarget, setLeft;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    /*
    //m_drive.setDefaultCommand(
    //  new RunCommand(()-> m_drive.tankDrive(left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    //);
    */
    
    m_drive.setDefaultCommand(
      new RunCommand(
        ()-> m_drive.tankDrive(left.getRawAxis(1), right.getRawAxis(1)), m_drive)
    );  
    
    
    /*
    m_drive.setDefaultCommand(new Drive(m_drive,
      ()->left.getRawAxis(1),
      ()->right.getRawAxis(5)
    ));
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    shootRun = new JoystickButton(left, 1);
      shootRun.whileHeld(new shoot(m_shoot));

    //shootRun = new JoystickButton(left, 1);
    //  shootRun.whileHeld(new StartEndCommand(() -> m_shoot.shoot(1), ()-> m_shoot.shoot(0), m_shoot));

    //shootRun = new JoystickButton(left, 1);
    //  shootRun.whileHeld(new bangBang(m_shoot, 1000));

    intakeOut = new JoystickButton(left, 6);
      intakeOut.whenPressed(new InstantCommand(()->m_intake.toggleSol(m_intake.intakeSOL), m_intake));

    switchShoot = new JoystickButton(left, 3);
      switchShoot.whenPressed(new InstantCommand(()->m_shoot.toggleSol(m_shoot.shooterSOL), m_shoot));

    intakeRun = new JoystickButton(left, 4);
      intakeRun.whileHeld(new StartEndCommand(() -> m_intake.runIntake(-.5), ()-> m_intake.runIntake(0), m_intake));

    feedRun = new JoystickButton(left, 5);
      feedRun.whileHeld(new StartEndCommand(() -> m_feed.runFeed(1), ()-> m_feed.runFeed(0), m_feed));

    turnTarget = new JoystickButton(right, 1);
      turnTarget.whileHeld(new TurnToTarget(m_drive, m_lime));
    setLeft = new JoystickButton(right, 14);
      setLeft.whileHeld(new StartEndCommand(()->m_drive.set(m_drive.front_L, .15), ()->m_drive.set(m_drive.front_L, 0), m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            m_drive.m_kinematics,
            5);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_drive.m_kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1)
            //new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    String trajectoryJSON = "paths/Unnamed.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        m_drive.m_kinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDrive(0, 0));
    //return new RunCommand(()-> m_drive.tankDrive(0.3, 0.3), m_drive).withTimeout(3);
  }
}
