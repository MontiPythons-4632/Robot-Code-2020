/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeefCake;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;

public class AimXDegrees extends CommandBase {
  /**
   * Creates a new TurnXDegrees.
   */
   private BeefCake beefCakeSubsystem;

  //  private double currX;
   private double currY;
   private double currZ;

   private double initZ;
   private double destZ;
   private boolean isTurning;
   private double initSpeed;
   private double degrees_to_turn;

  public AimXDegrees(BeefCake subsystem, double degrees) {  //  cant have a double speed because the robot will slide
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
    this.beefCakeSubsystem = subsystem;
    this.initSpeed = 0.6;
    this.degrees_to_turn = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.initZ = this.beefCakeSubsystem.getCurrentAngle();
    this.destZ = this.initZ + this.degrees_to_turn;
    SmartDashboard.putNumber("Target Turn Angle", this.destZ);
    System.out.println("Turning");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.currZ = this.beefCakeSubsystem.getCurrentAngle();

    // Left is 1 Right is -1
    double direction = 1;
    if ( this.currZ < this.destZ ) {
      direction = -1;
    }

    // As currX approaches destX, THIS goes from 0 to 1, acutalSpeed goes from 1 to 0.5
    //                            v---^^^^---------------------------------------------- -----------v
   //double actualSpeed = Math.max(1 - Math.abs((this.currX - this.initX) / (this.destX - this.initX)) , 0.5);
    double actualSpeed = 0.6;

    SmartDashboard.putNumber("Current Turn Angle",this.currY);

    this.beefCakeSubsystem.moveAngle(actualSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ( Math.abs( this.currZ - this.destZ) < 1.0) {
      SmartDashboard.putString("Target Turn Angle", "Not Turning");
      this.beefCakeSubsystem.stopAngle();
      return true;
    }
    return false;
  }
}
