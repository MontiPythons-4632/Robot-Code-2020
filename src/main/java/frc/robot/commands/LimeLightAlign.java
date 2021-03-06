/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.networktables.*;
import frc.robot.subsystems.BeefCake;
import frc.robot.subsystems.Drive;
import frc.robot.commands.TurnXDegrees;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightAlign extends CommandBase {
  /**
   * Creates a new LimeLightAlign.
   */
  private Drive driveSubsystem;
  private double targetAquired;
  private double currXOffset;
  private double currXHeading;
  private double destXHeading;
  private double initDriveSpeed;
  private double actualSpeed;
  private double timer;

  public LimeLightAlign(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = drive;
    this.initDriveSpeed = 0.75;
  
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // System.out.println("Running LimeLightAlign");
    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // Turn light on
    this.driveSubsystem.setLimeLightDetectionMode();
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    this.destXHeading = this.driveSubsystem.getCurrentHeading() - NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // System.out.println("Limelight XOffset: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));

    this.driveSubsystem.setAimingMode();
    
    // System.out.println("Align DestXHeading: " + this.destXHeading);
    
    this.timer = 0;

    SmartDashboard.putString("Mode", "Aligning");
    if (this.targetAquired == 1) {
      SmartDashboard.putBoolean("Target Aquired", true);
    } else {
      SmartDashboard.putBoolean("Target Aquired", false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // this.currXOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    this.currXHeading = this.driveSubsystem.getCurrentHeading();
    this.currXOffset = this.currXHeading - this.destXHeading;
    // System.out.println("Robot Heading: " + this.currXHeading + " | Target Heading: " + this.destXHeading);
    // System.out.println("Align currXOffset: " + this.currXOffset);

    // Left is 1 Right is -1
    double direction = 1;
    if ( this.currXOffset < 0 ) {
            direction = -1;
    }
    
    // As currXOffset approaches 0, THIS goes from +-1 to 0
    this.actualSpeed = Math.min(0.6 + Math.abs(this.currXOffset / 5 * 0.05), this.initDriveSpeed);
    // System.out.println("Align Speed: " + this.actualSpeed);
    this.driveSubsystem.arcade(0, direction*this.actualSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(this.currXOffset) < 1.0)  {
      this.timer++;
      // System.out.println("timer--> " + timer);
    } else {
    this.timer = 0;
    }

    if (this.timer >= 25) {
      SmartDashboard.putString("Mode", "Ready!");
      // this.driveSubsystem.setLimeLightNormalMode();
      return true;
    }

    return false;
  }
}
