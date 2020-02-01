
//  PAGE 676 in CommandBasedProgramming.pdf

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnXDegrees;
import frc.robot.commands.DriveForwardXFeet;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestSequentialCommandGroup extends SequentialCommandGroup {
/**
   * Creates a new TestSequentialCommandGroup.
   */
  private static Drive drive;
  private TurnXDegrees turnXDegrees;
  private DriveForwardXFeet driveForwardXFeet;

  public TestSequentialCommandGroup() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(
      new TurnXDegrees(drive, -45),
      new DriveForwardXFeet(drive, 6.0, 0.8)
      );
  }
}
