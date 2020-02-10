/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.Joystick.ButtonType;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Set;

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
  private final ColorWheel colorWheel = new ColorWheel();

  // The robot's commands are defined here...
  // private BeefCakeJoystickAngle beefCakeJoystickAngle = new BeefCakeJoystickAngle(beefCake, beefCakeJoystick);
  // private final DriveForwardXFeet driveXFeet = new DriveForwardXFeet(drive, 10.0, 0.5);
  // private final DriveJoystick driveJoystickCommand = new DriveJoystick(drive, driveJoystick);
  // private StartBeefCakeFeed startBeefCakeFeed = new StartBeefCakeFeed(beefCake);

  // The container for the robot.  Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    configureButtonBindings();

    this.drive.setDefaultCommand(
      new RunCommand(() -> drive.arcade(driveJoystick.getY()*-1.0, driveJoystick.getX()), 
                           drive
                    )
    );

    this.beefCake.setDefaultCommand(
      new RunCommand(() -> beefCake.angleJoystick(beefCakeJoystick.getY()*-0.75),
                           beefCake
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

    /*-------------/ BEEFCAKE /-------------*/

    //  Activate/Deactivate the feeder
    new JoystickButton(this.beefCakeJoystick, 1)
      .whenPressed(new InstantCommand(this.beefCake::feederOn, this.beefCake))
      .whenReleased( this.beefCake::feederOff)
    ;

    //  Angles the launcher with buttons
    // new JoystickButton(this.beefCakeJoystick, 3)
    //   .whenPressed(this.beefCake::adjustAngleDown)
    //   .whenReleased(this.beefCake::stopAngle)
    // ;
    
    // new JoystickButton(this.beefCakeJoystick, 2)
    //   .whenPressed(this.beefCake::adjustAngleUp)
    //   .whenReleased(this.beefCake::stopAngle)
    // ;

    //  Activate/Deactivate the launcher wheels
    new JoystickButton(this.beefCakeJoystick, 6)
      .whenPressed(this.beefCake::launcherOn)
    ;

    new JoystickButton(this.beefCakeJoystick, 7)
      .whenPressed(this.beefCake::launcherOff)
    ;
    
    new JoystickButton(this.beefCakeJoystick, 8)
      .whenPressed(this.beefCake::climbUp)
      .whenReleased(this.beefCake::climbOff);

    new JoystickButton(this.beefCakeJoystick, 9)
      .whenPressed(this.beefCake::climbDown)
      .whenReleased(this.beefCake::climbOff);

    /*-------------/ DRIVE /-------------*/

    // Control the intake
    new JoystickButton(this.driveJoystick, 1)
      .whenPressed(this.beefCake::intakeOn)
      .whenReleased(this.beefCake::intakeOff);
    ;

    //  Change robot speed
    new JoystickButton(this.driveJoystick, 2)
      .whenPressed(this.drive::setLimitSlow)
      .whenReleased(this.drive::setLimitNorm)
    ;

    new JoystickButton(this.driveJoystick, 3)
      .whenPressed(this.drive::setLimitFast)
      .whenReleased(this.drive::setLimitNorm)
    ;
    
    //  Turns the robot left or right 45 degrees ( +degrees is left, -degrees is right)
    new JoystickButton(this.driveJoystick, 4)
      .whenPressed(new TurnXDegrees(this.drive, 25.0))
    ;
   
    new JoystickButton(this.driveJoystick, 5)
      .whenPressed(new TurnXDegrees(this.drive, -25.0))
    ;
  
    //  Reverses the Intake in case of a stuck ball
    new JoystickButton(this.driveJoystick, 7)
      .whenPressed(this.beefCake::intakeReverse)
      .whenReleased(this.beefCake::intakeOff);
    ;

    //  Changes the drive direction for Intake and Aiming
    new JoystickButton(this.driveJoystick, 8)
      .whenPressed(new InstantCommand(this.drive::setIntakeMode, this.drive))
    ;

    new JoystickButton(this.driveJoystick, 9)
      .whenPressed(new InstantCommand(this.drive::setAimingMode, this.drive))
    ;

    //  Runs the DriveForwardXFeet command once
    // new JoystickButton(this.driveJoystick, 10)
    //   .whenPressed(new DriveForwardXFeet(this.drive, 6.5, 0.8))
    // ;

    //  Runs the LimeLightAlign command
    new JoystickButton(this.driveJoystick, 10)
      .whenPressed(new AutoShoot(this.beefCake, this.drive) 
      )
    ;

    //  Runs TestSequentialCommandGroup command once
    new JoystickButton(this.driveJoystick, 11)
      .whenPressed(
        new SequentialCommandGroup(
          new DriveForwardXFeet(drive, 6.5, 0.8), 
          new TurnXDegrees(drive, 62),
          new DriveForwardXFeet(drive, 6.5, 0.8), 
          new TurnXDegrees(drive, 62),
          new DriveForwardXFeet(drive, 6.5, 0.8), 
          new TurnXDegrees(drive, 62),
          new DriveForwardXFeet(drive, 6.5, 0.8), 
          new TurnXDegrees(drive, 62)
        )
      )
    ;
  }
    
  //  Use this to pass the autonomous command to the main {@link Robot} class.
  //  @return the command to run in autonomous
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    System.out.println("getting Autonomous Command");
    return createPathCommand();

    
  }

  private Command createPathCommand() {

    Path trajectoryPath1;
    Trajectory trajectory1;

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

    // Import the trajectory
    String trajectoryJSON = "/home/lvuser/deploy/paths/Position 1 Shoot.wpilibb.json";
    try {
      trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory1,
        drive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        this.drive::tankDriveVolts,
        this.drive
      );
      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return null;

  }
}
