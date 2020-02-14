/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public final class DriveConstants {

        public static final boolean DEBUG = false;

        public static final double kDriveSlow = 0.5;
        public static final double kDriveNorm = 0.7;
        public static final double kDriveFast = 1.0;
        public static final double kTurnSlow = 0.8;
        public static final double kTurnNorm = 0.7;
        public static final double kTurnFast = 0.6;

        public static final double cpr = 214; // if am-3314a
        public static final double whd = 6; // for 6 inch wheel
        
        // For Limelight Distance calculations
        public static final double kCameraAngle = 32.77;
        public static final double kCameraHeight = 0.24; // 9.5 inches ground to lens
        public static final double kTargetHeight = 2.74; // height to center of target in Meters
        
    }

    // Constants for BeefCake Subsystem
    public final class BeefCakeConstants {

        public static final boolean DEBUG = true;

        public static final double kFeederSpeed = 0.6;
        public static final double kAngleSpeed = 0.7;
        public static final double kLauncherSpeed = 0.7;

        public static final double kIntake = 0.5;


    }
}
