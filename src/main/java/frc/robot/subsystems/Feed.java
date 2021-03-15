// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feed extends SubsystemBase {
  /** Creates a new Feed. */
  private CANSparkMax feed;
  private BooleanSupplier cShoot = ()-> false;

  public Feed() {
    feed = new CANSparkMax(Constants.FEED_MOTOR, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFeed(double speed){
    feed.set(speed);
  }

  public void autoShoot(double speed){
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter System");
    NetworkTableEntry canShoot = tab.add("Shoot?", cShoot.getAsBoolean()).getEntry();
    boolean shoot = canShoot.getBoolean(false);

    if(shoot){
      feed.set(speed);
    }else{
      feed.set(0);
    }
  }
}
