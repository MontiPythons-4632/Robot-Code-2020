/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;
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
    private double initXOffset;
    private double initSpeed;
    private double actualSpeed;

  public LimeLightAlign(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = drive;
    this.initSpeed = 0.75;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    this.driveSubsystem.setAimingMode();

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
    System.out.println(this.currXOffset);
    this.currXOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    // Left is 1 Right is -1
    double direction = 1;
    if ( this.currXOffset < 0 ) {
            direction = -1;
    }
    
    // As currXOffset approaches 0, THIS goes from +-1 to 0
    //                                  v---^^^^----------------------------v
    // this.actualSpeed = this.initSpeed * (this.currXOffset / this.initXOffset);
    this.actualSpeed = Math.min(0.5 + Math.abs(this.currXOffset / 5 * 0.05), this.initSpeed - .2);

    this.driveSubsystem.arcade(0, direction*this.actualSpeed);

    SmartDashboard.putString("Mode", "Ready!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Math.abs(this.currXOffset) < 1.0 )  {
      return true;
    }
    return false;
  }
}
