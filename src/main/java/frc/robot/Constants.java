/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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

        public static final boolean DEBUG = false;

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
        public static final double ksVolts = 0.745;
        public static final double kvVoltSecondsPerMeter = 0.0751;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0113;
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds

        public static final double kRamseteB = 2;

        public static final double kRamseteZeta = 0.7;
    }

    // Constants for BeefCake Subsystem
    public final class BeefCakeConstants {

        public static final boolean DEBUG = true;

        public static final double kFeederSpeed = 0.6;
        public static final double kAngleSpeed = 0.7;
        public static final double kLauncherSpeed = 0.7;

        public static final double kIntake = 0.5;
    }

    public final class AutoDriveConstants{
        public static final String path = "/home/lvuser/deploy/paths";
    }
}
