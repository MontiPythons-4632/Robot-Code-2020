/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.AutonomousPath;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;


/**
 * Add your docs here.
 */
public class CreateAutoPath {
    // Variables
    private Drive driveSubsystem;
    private AutonomousPath path;
    private Path trajectoryJSON;
    private Trajectory trajectory;
    private Boolean failedToLoad;
    private RamseteCommand ramseteCommand;

    public void initialize(Drive drive, AutonomousPath path) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveSubsystem = drive;
        this.path = path;

        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                        DriveConstants.kvVoltSecondsPerMeter,
                                        DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);
    
        // Import the trajectory
        this.trajectoryJSON = Filesystem.getDeployDirectory().toPath().resolve(Constants.AutoDriveConstants.path + "/" + this.path.getFilename());
        try {
          this.trajectory = TrajectoryUtil.fromPathweaverJson(this.trajectoryJSON);
          this.ramseteCommand = new RamseteCommand(
            this.trajectory,
            this.driveSubsystem::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            DriveConstants.kDriveKinematics,
            this.driveSubsystem::tankDriveVolts,
            this.driveSubsystem
          );        
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
          this.failedToLoad = true;
        }
    }

}
