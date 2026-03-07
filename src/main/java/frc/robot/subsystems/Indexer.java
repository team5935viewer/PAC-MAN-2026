// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private SparkMax hopperDrive;
  private SparkMax leftDrive;
  private SparkMax rightDrive;

  /** Creates a new Indexer. */
  public Indexer() {
    this.hopperDrive = new SparkMax(0, MotorType.kBrushless);
    this.leftDrive = new SparkMax(0, MotorType.kBrushless);
    this.rightDrive = new SparkMax(0, MotorType.kBrushless);


  }

  public void driveHopper(double percentOutput) {
    hopperDrive.set(percentOutput);
  }

  public void driveLR(double percentOutput) {
    leftDrive.set(percentOutput);
    rightDrive.set(percentOutput);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
