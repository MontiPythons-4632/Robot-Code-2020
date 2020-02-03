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
   private double initSpeed;
   private double degrees_to_turn;

  public TurnXDegrees(Drive subsystem, double degrees) {  //  cant have a double speed because the robot will slide
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
    this.driveSubsystem = subsystem;
    this.initSpeed = 0.6;
    this.degrees_to_turn = degrees;

    // this.degrees_to_turn = this.degrees_to_turn;
    // this.initX = this.driveSubsystem.getCurrentHeading();

    // this.destX = this.initX + this.degrees_to_turn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // this.isTurning = false;
    // this.initX = this.driveSubsystem.getCurrentHeading();
    // this.destX = this.initX + this.degrees_to_turn;
    // SmartDashboard.putNumber("Target Turn Angle", this.destX);

    this.initX = this.driveSubsystem.getCurrentHeading();
    this.destX = this.initX + this.degrees_to_turn;
    // System.out.println("Destination X: " + this.destX);
    SmartDashboard.putNumber("Target Turn Angle", this.destX);
    System.out.println("Turning");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.currX = this.driveSubsystem.getCurrentHeading();

    // Left is 1 Right is -1
    double direction = 1;
    if ( this.currX < this.destX ) {
            direction = -1;
    }

    // As currX approaches destX, THIS goes from 0 to 1, speedLimiter goes from 1 to 0
    //                        v---^^^^------------------------------------------------------v
    double speedLimiter = 1 - Math.abs((this.currX - this.initX) / (this.destX - this.initX));
    double actualSpeed = (this.initSpeed * (speedLimiter)) + 0.5;
    // System.out.println(actualSpeed);

    SmartDashboard.putNumber("Current Turn Angle",this.currX);

    this.driveSubsystem.arcade(0, direction*actualSpeed);

    // if( Math.abs(this.currX - this.destX) < 10 ) {
    //   speedLimit = DriveConstants.kDriveSlow;
    // }
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
