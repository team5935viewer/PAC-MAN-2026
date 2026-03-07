// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SparkMax intakeArm;
  SparkClosedLoopController intakeArmPID = intakeArm.getClosedLoopController();
  private SparkMax intakeWheels;

  private final double downPos = 5.0; // 0 = full up then this value is full down.

  /** Creates a new Intake. */
  public Intake() {
    intakeArm = new SparkMax(0, MotorType.kBrushless);
    intakeWheels = new SparkMax(0, MotorType.kBrushless);

    SparkMaxConfig intakeArmConfig = new SparkMaxConfig();

    intakeArmConfig.closedLoop
      .p(0.0)
      .i(0.0)
      .d(0.0)
      .outputRange(0.0, 0.0);

    intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  


  }

  public void setIntakeArm(double setpoint) {
    intakeArmPID.setSetpoint(setpoint, ControlType.kPosition);
  }

  public void driveIntake(double percentOutput) {
    intakeWheels.set(percentOutput);
  }

  public void setIntakeUP() {
    intakeArmPID.setSetpoint(0.0, ControlType.kPosition);
  }

  public void setIntakeDOWN() {
    intakeArmPID.setSetpoint(downPos, ControlType.kPosition);
  }

  public Double getArmPos() {
    return intakeArm.getEncoder().getPosition();
  }

  public void zero() {
    intakeArm.getEncoder().setPosition(0);
  }

  public void stop() {
    intakeWheels.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
