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
import frc.robot.Constants;
import frc.robot.commands.AimXDegrees;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightAim extends CommandBase {
  /**
   * Creates a new LimeLightAim.
   */
  private BeefCake beefCakeSubsystem;
  private double targetAquired;
  private double currZOffset;
  private double currZPitch;
  private double destZPitch;
  private double angleSpeed;
  private double direction;
  
  public LimeLightAim(BeefCake beefCake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.beefCakeSubsystem = beefCake;
    this.angleSpeed = 0.6;

    addRequirements(beefCake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // System.out.println("Running LimeLightAim");
    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    // this.driveSubsystem.setLimeLightDetectionMode();

    // this.destZPitch = this.beefCakeSubsystem.getCurrentAngle() - NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // this.destZPitch = 45.0;
    this.destZPitch = Constants.BeefCakeConstants.testDestZPitch;
    // this.destZPitch = SmartDashboard.getNumber("testDestZPitch", 0);

    // this.driveSubsystem.setAimingMode();

    SmartDashboard.putString("Mode", "Aiming");
    if (this.targetAquired == 1) {
      SmartDashboard.putBoolean("Target Aquired", true);
    } else {
      SmartDashboard.putBoolean("Target Aquired", false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // this.currZOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    this.currZPitch = this.beefCakeSubsystem.getCurrentAngle();
    this.currZOffset = this.currZPitch - this.destZPitch;
    System.out.println("BeefCake Pitch: " + this.currZPitch + " | Target Pitch: " + this.destZPitch);
    System.out.println("BeefCake Align currOffset: " + this.currZOffset);

    // Down is 1 Up is -1
    this.direction = 0.8;
    if ( this.currZOffset < 0 ) {
            this.direction = -0.5;
    }

    this.beefCakeSubsystem.moveAngle(direction * this.angleSpeed);
    System.out.println("Angling");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(this.currZOffset) < 1.0)  {
      System.out.println("Stopping Angling! --> Destination Reached");

      if (this.direction < 0) {
        this.beefCakeSubsystem.moveAngle(0.1);
        System.out.println("Adding Backlash");
        Timer.delay(0.05);
      }

      this.beefCakeSubsystem.stopAngle();
      return true;
    }

    if ( (this.currZPitch < 0 && this.direction > 0) || 
         (this.currZPitch > Constants.BeefCakeConstants.kAngleRangeMax && this.direction < 0)
       ) {
        System.out.println("Stopping Angling! --> Limit Reached");
        this.beefCakeSubsystem.stopAngle();
        return true;
    }

    return false;
  }
}
