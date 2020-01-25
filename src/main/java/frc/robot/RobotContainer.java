/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.Joystick.ButtonType;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.buttons.Button;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driveJoystick = new Joystick(0);
  private Joystick beefCakeJoystick = new Joystick(1);

  // The robot's subsystems are defined here...
  private final Drive drive = new Drive();
  private final BeefCake beefCake = new BeefCake();

  // The robot's commands are defined here...
  private BeefCakeJoystickAngle beefCakeJoystickAngle = new BeefCakeJoystickAngle(beefCake, beefCakeJoystick);
  private final DriveJoystick driveJoystickCommand = new DriveJoystick(drive, driveJoystick);
  private final DriveForwardXFeet driveXFeet = new DriveForwardXFeet(drive, 10.0);
  private StartBeefCakeFeed startBeefCakeFeed = new StartBeefCakeFeed(beefCake);

  // The container for the robot.  Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    this.driveXFeet.withTimeout(5.0);
    configureButtonBindings();

    this.drive.setDefaultCommand(
      new RunCommand(() -> drive.arcade(driveJoystick.getY()*-1.0,
                                        driveJoystick.getX()
                                       ), 
                           drive
                    )
    );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  
    System.out.println("Setting Bindings");

    new JoystickButton(this.beefCakeJoystick, 1)
       .whenPressed(new InstantCommand(this.beefCake::feederOn, this.beefCake))
       .whenReleased( this.beefCake::feederOff);

    // new JoystickButton(this.beefCakeJoystick, 1)
    //     .whenPressed( new ConditionalCommand(
    //                       new InstantCommand(this.beefCake::feederOn, this.beefCake),
    //                       new InstantCommand(),
    //                       this.beefCake.isLauncherAtSpeed()
    //                   )
        // )
        // .whenReleased( this.beefCake::feederOff);

    //  Change robot speed limit. Based on buttons 2 and 3
    new JoystickButton(this.driveJoystick, 3)
      .whenPressed(this.drive::setLimitFast)
      .whenReleased(this.drive::setLimitNorm);
    
    new JoystickButton(this.driveJoystick, 2)
      .whenPressed(this.drive::setLimitSlow)
      .whenReleased(this.drive::setLimitNorm);

    //  Activate/Deactivate the launcher wheels
    new JoystickButton(this.beefCakeJoystick, 6)
      .whenPressed(this.beefCake::launcherOn);

    new JoystickButton(this.beefCakeJoystick, 7)
      .whenPressed(this.beefCake::launcherOff);

    //  Angles the launcher with buttons
    // new JoystickButton(this.beefCakeJoystick, 10)
    //   .whenPressed(this.beefCake::adjustAngleUp)
    //   .whenReleased(this.beefCake::stopAngle);
    
    // new JoystickButton(this.beefCakeJoystick, 11)
    //   .whenPressed(this.beefCake::adjustAngleDown)
    //   .whenReleased(this.beefCake::stopAngle);


  }      
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    System.out.println("getting Autonomous Command");
    return driveXFeet;
  }
}
