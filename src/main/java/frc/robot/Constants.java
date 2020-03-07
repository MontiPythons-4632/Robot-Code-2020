/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Constants for Drive Subsystem
    public final static class DriveConstants {
        
        // Make these true to print debug statements in console
        // 0 is off
        // 1 is status messages
        // use higher numbers for more detail
        public static final int kDebugDriveAll = 0;
        public static final int kDebugDrive = kDebugDriveAll;
        public static final int kDebugLimeLight = kDebugDriveAll;


        public static final double kDriveSlow = 0.5;
        public static final double kDriveNorm = 0.7;
        public static final double kDriveFast = 1.0;
        public static final double kTurnSlow = 0.8;
        public static final double kTurnNorm = 0.7;
        public static final double kTurnFast = 0.6;

        public static final double cpr = 214; // if am-3314a
        public static final double whd = 6; // for 6 inch wheel

        public static final boolean kGyroReversed = false;

        // These all need to be identified
        public static final double ksVolts = 0.974;
        public static final double kvVoltSecondsPerMeter = 2.57;
        public static final double kaVoltSecondsSquaredPerMeter = 0.402;
        public static final double kMaxSpeedMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kTrackwidthMeters = 0.874;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        
        // For Limelight Distance calculations
        public static final double kCameraAngle = 28.7937;
        public static final double kCameraHeight = 1.0541; // meters from ground to lens
        public static final double kTargetHeight = 2.7432; // height to center of target in Meters
    }

    // Constants for BeefCake Subsystem
    public final static class BeefCakeConstants {

        // Make these true to print debug statements in console
        // 0 is off
        // 1 is status messages
        // use higher numbers for more detail
        public static final int kBeefCakeAll = 0;
        public static final int kDebugBeefCakeFeeder = kBeefCakeAll;
        public static final int kDebugBeefCakeLauncher = kBeefCakeAll;
        public static final int kDebugBeefCakeIntake = kBeefCakeAll;
        public static final int kDebugBeefCakeAngle = kBeefCakeAll;
        public static final int kDebugBeefCakeClimber = kBeefCakeAll;

        public static final double kStartingAngle = -75.87;
        public static final double kFeederSpeed = 0.5;
        public static final double kAngleSpeed = 0.7;
        public static final double kAngleRangeMax = 84;

        public static final double kLauncherSpeed = 0.5;
        public static final double testDestZPitch = 45;        

        public static final double kIntake = 0.4;
        public static final double kColorSelectSpeed = 0.2;

        
        // Infinite Reacharge Colors
        public static final Color kBlueTarget = ColorMatch.makeColor(0.17, 0.45, 0.35);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.22, 0.52, 0.25);
        public static final Color kRedTarget = ColorMatch.makeColor(0.36, 0.43, 0.2);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.3, 0.53, 0.17);
    }

    public final static class AutoDriveConstants{
        public static final Double kAutoTimeoutSeconds = 4.0;
        public static final Double kAutoShootTimeSeconds = 3.0;
        public static final String path = "/home/lvuser/deploy/paths";
    }
}
