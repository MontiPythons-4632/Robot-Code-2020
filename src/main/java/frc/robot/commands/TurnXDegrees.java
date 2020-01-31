/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;

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
   private double degrees_to_turn;

  public TurnXDegrees(Drive subsystem, double degrees, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
    this.driveSubsystem = subsystem;
    this.speed = speed;
    this.degrees_to_turn = degrees;
    this.degrees_to_turn = this.degrees_to_turn;
    this.initX = this.driveSubsystem.getCurrentHeading();

    this.destX = this.initX + this.degrees_to_turn;

    SmartDashboard.putNumber("Target Turn Angle", this.destX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // this.isTurning = false;
    // this.initX = this.driveSubsystem.getCurrentHeading();
    // this.destX = this.initX + this.degrees_to_turn;
    // SmartDashboard.putNumber("Target Turn Angle", this.destX);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 
    double speedLimit = DriveConstants.kDriveFast;

    // Left is 1 Right is -1
    double direction = 1;
    if ( this.currX < this.destX ) {
            direction = -1;
    }

    //slow as we get closer
    if( Math.abs(this.currX - this.destX) < 10 ) {
      speedLimit = DriveConstants.kDriveSlow;
    }

    this.currX = this.driveSubsystem.getCurrentHeading();
    
    SmartDashboard.putNumber("Current Turn Angle",this.currX);

    this.driveSubsystem.arcade(this.speed*speedLimit, direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( Math.abs( this.currX - this.destX) < 1.0) {
      SmartDashboard.putString("Target Turn Angle", "Not Turning");
      return true;
    }
    return false;
  }
}
