/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeefCake;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.BeefCakeConstants;


public class BeefCakeJoystickAngle extends CommandBase {
  /**
   * Creates a new BeefCakeJoystickAngle.
   */

  private BeefCake beefCake;
  private Joystick joystick;

  public BeefCakeJoystickAngle(BeefCake subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.beefCake = subsystem;
    this.joystick = joystick;

    addRequirements(beefCake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = this.joystick.getY()*BeefCakeConstants.kFeederSpeed;

    if( speed > 0.1 ) {
      beefCake.angleJoystick(speed);
    }

    if( speed < -0.1 ) {
      beefCake.angleJoystick(0.25*speed);
    }

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
