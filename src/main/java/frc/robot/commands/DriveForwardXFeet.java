/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardXFeet extends CommandBase {
  /**
   * Creates a new DriveForwardXFeet.
   */
  private final Drive m_drive;

  double distanceToTravelInFeet;

  public DriveForwardXFeet(Drive subsystem, double xFeet) {
    m_drive = subsystem;
    distanceToTravelInFeet = xFeet;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    withTimeout(0.5 * distanceToTravelInFeet); // 0.5 seconds per foot
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcade(0.5, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
