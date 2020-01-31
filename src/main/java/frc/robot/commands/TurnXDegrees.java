/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnXDegrees extends CommandBase {
  /**
   * Creates a new TurnXDegrees.
   */
   private Drive driveSubsystem;

   private double currX;
   private double currY;
   private double currZ;

   private double initX;
   private double destX;
   private boolean isTurning;
   private double speed;

  public TurnXDegrees(Drive subsystem, double degrees, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
    this.driveSubsystem = subsystem;
    this.initX = this.driveSubsystem.getCurrentHeading();
    this.destX = this.initX + degrees;
    this.speed = speed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.isTurning = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double direction = 1;

    this.currX = this.driveSubsystem.getCurrentHeading();
    
    if ( this.currX > this.destX ) {
        direction = -1;
    }

    this.driveSubsystem.arcade(this.speed, direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( Math.abs( this.currX - this.initX) < 0.1 ) {
        return true;
    }
    return false;
  }
}
