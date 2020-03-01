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
  
  public LimeLightAim(BeefCake beefCake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.beefCakeSubsystem = beefCake;
    this.angleSpeed = 0.3
    ;

    addRequirements(beefCake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Running LimeLightAim");
    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    // this.driveSubsystem.setLimeLightDetectionMode();

    // this.destZPitch = this.beefCakeSubsystem.getCurrentPitch() - NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    this.destZPitch = 45.0;

    // this.driveSubsystem.setAimingMode();

    // SmartDashboard.putString("Mode", "Aligning");
    // if (this.targetAquired == 1) {
    //   SmartDashboard.putBoolean("Target Aquired", true);
    // } else {
    //   SmartDashboard.putBoolean("Target Aquired", false);
    // }
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
    double direction = 1.0;
    if ( this.currZOffset < 0 ) {
            direction = -1.0;
    }

    this.beefCakeSubsystem.moveAngle(direction * this.angleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(this.currZOffset) < 2.0)  {
      this.beefCakeSubsystem.stopAngle();
      return true;
    }

    if (this.currZPitch < 0 || 
      this.currZPitch > Constants.BeefCakeConstants.kAngleRangeMax) {
        this.beefCakeSubsystem.stopAngle();
        return true;
    }

    return false;
  }
}
