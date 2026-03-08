// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexerCMD extends Command {

  private Indexer indexer;

  private double driveHopper; // These values are configured when the command is 
  private double driveLR;     // constructed. Values are between -1.0 to 1.0

  /** Creates a new IndexerCMD. */
  public IndexerCMD(Indexer indexer, double driveHopper, double driveLR) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    this.indexer = indexer;
    this.driveHopper = driveHopper;
    this.driveLR = driveLR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.driveHopper(driveHopper);
    indexer.driveLR(driveLR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
