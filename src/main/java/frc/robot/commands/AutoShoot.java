/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.BeefCake;
import frc.robot.subsystems.Drive;
import frc.robot.commands.TurnXDegrees;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoShoot.
   */

  public AutoShoot(BeefCake beefCakeSubSystem, Drive driveSubSystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
        

    super( 
      new InstantCommand(beefCakeSubSystem::intakeFlipOut, beefCakeSubSystem),
      new InstantCommand(beefCakeSubSystem::launcherOn, beefCakeSubSystem),
      new InstantCommand(driveSubSystem::setAimingMode, driveSubSystem),
      new ParallelCommandGroup(
        new LimeLightAlign(driveSubSystem),
        new LimeLightAim(beefCakeSubSystem)
      ),
      new LimeLightAlign(driveSubSystem),
      new InstantCommand(beefCakeSubSystem::feederOn, beefCakeSubSystem)
      );

  }
}
