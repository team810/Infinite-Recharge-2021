// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //JOYSTICKS
	public static final int GAMEPAD = 0;
	public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 1;
    //JOYSTICK BUTTONS
    
    public static final int INTAKE_BTN = 1;
    public static final int FAR_SHOOT_BTN = 14;
    public static final int INTAKE_SOL_BTN = 3;
    public static final int CLOSE_SHOOT_BTN = 4;
    public static final int SWITCH_SHOOT = 5;
    public static final int CPSOL_BTN = 6;
    public static final int FEEDER_BTN = 13;
    public static final int ARMSOL_BTN = 10;
    public static final int ARMMOTOR_BTN = 9;
    public static final int WINCH_BTN = 10;
    public static final int CP_BTN = 11;
    public static final int DRIVE_MODE_BTN = 6;
    public static final int FEED_REVERSE_BTN = 12;


    //CANSPARKMAX
	public static final int FRONTL = 3;
	public static final int BACKL = 4;
	public static final int FRONTR = 1;
    public static final int BACKR = 2;

    public static final int SHOOT_MOTOR = 8;
    public static final int SHOOT_SLAVE = 9;
    public static final int FEED_MOTOR = 7;
    public static final int INTAKE_MOTOR = 5;
    
    public static final double RADIUS = 7.56 / 2.0;

    public static final double CIRCUMFERENCE = Math.PI * 2.0 * RADIUS;

    //DOUBLE SOLENOIDS
    public static final int INTAKE_FORWARD = 4;
    public static final int INTAKE_REVERSE = 5;

    public static final int SHOOTER_FORWARD = 2;
    public static final int SHOOTER_REVERSE = 3;
  
    public static final boolean kGyroReversed = false;
	public static final double kTurnToleranceDeg = 3;
    public static final double kTurnRateToleranceDegPerS = 3;
    
    public static final int ULTRASONIC = 0;

    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-bulls");
        
    public static final NetworkTableEntry tx = table.getEntry("tx");
    public static final NetworkTableEntry ty = table.getEntry("ty");
    public static final NetworkTableEntry ta = table.getEntry("ta");
    public static final NetworkTableEntry tv = table.getEntry("tv");
    

    public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
    public static final NetworkTableEntry camMode = table.getEntry("camMode");
    public static final NetworkTableEntry pipeline = table.getEntry("pipeline");
    public static final NetworkTableEntry stream = table.getEntry("stream");
    
    public static final double GEAR_RATIO = 12.75;
    public static final double WHEEL_DIAMETER = 7.56;

    //AUTONOMOUS
    public static final double ksVolts = 0.184;
    public static final double kvVoltSecondsPerMeter = 2.74;
    public static final double kaVoltSecondsSquaredPerMeter = 0.319;

    public static final double kPDriveVel = 12.6;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kvVoltSecondsPerRadian = 2.74;
    public static final double kaVoltSecondsSquaredPerRadian = 0.321;

    public static final double kRamseteB = 2; // dont change this and the val below
    public static final double kRamseteZeta = 0.7;


    //TRAJGEN
    public static final double TRACK_WIDTH_METERS = 1.135908083891128;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = 
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian
        );
}
