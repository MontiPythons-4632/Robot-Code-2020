/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;

// import java.util.concurrent.atomic.DoubleAccumulator;

// import com.ctre.phoenix.motorcontrol.Faults;
// import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.*;

public class Drive extends SubsystemBase {
  // Creates a new Drive.

  private WPI_TalonSRX leftFront;
  private WPI_VictorSPX leftBack;
  private SpeedControllerGroup left;
  private WPI_TalonSRX rightFront;
  private WPI_VictorSPX rightBack;
  private SpeedControllerGroup right;
  private DifferentialDrive differentialDrive;
  private double speedLimit;
  private double turnLimit;

  private double driveInvert = 1.0;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private double leftDistanceTraveled;
  private double rightDistanceTraveled;

  // Variables for Pigeon 9DOF Sensor
  private PigeonIMU pigeon;
  private double curX;
  // private double curY;
  // private double curZ;
  // private double curCompass;

  //  Limelight variables
  private double targetAquired;
  // private double horizontalOffset;
  private double verticalOffset;


   // Odometry class for tracking robot pose
   private DifferentialDriveOdometry odometry;

   public Drive() {
    leftFront = new WPI_TalonSRX(1);
    leftBack = new WPI_VictorSPX(2);
    left = new SpeedControllerGroup(leftFront, leftBack);
    left.setInverted(false);
    addChild("Left", left);

    rightFront = new WPI_TalonSRX(3);
    rightBack = new WPI_VictorSPX(4);
    right = new SpeedControllerGroup(rightFront, rightBack);
    right.setInverted(false);
    addChild("Right", right);

    differentialDrive = new DifferentialDrive(left, right);
    addChild("Differential Drive", differentialDrive);
    differentialDrive.setSafetyEnabled(false);
    differentialDrive.setExpiration(0.3);
    differentialDrive.setMaxOutput(1.0);

    this.speedLimit = DriveConstants.kDriveNorm;
    this.turnLimit = DriveConstants.kTurnNorm;

    // factory default values
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    // [4] adjust sensor phase so sensor moves positive when Talon LEDs are green
    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);
    rightBack.setSensorPhase(true);
    leftBack.setSensorPhase(true);

    // Set up encoder
    this.leftEncoder = new Encoder(6, 7);
    this.leftEncoder.reset();
    this.leftEncoder.setDistancePerPulse(Math.PI * DriveConstants.whd / DriveConstants.cpr); // distance per pulse is pi* (wheel diameter / counts per
                                                          // revolution)
    this.leftEncoder.setReverseDirection(true);

    this.rightEncoder = new Encoder(8, 9);
    this.rightEncoder.reset();
    this.rightEncoder.setDistancePerPulse(Math.PI * DriveConstants.whd / DriveConstants.cpr); // distance per pulse is pi* (wheel diameter / counts per
                                                           // revolution)
    this.rightEncoder.setReverseDirection(false);

    // Initialize the Pigeon 9DOF
    pigeon = new PigeonIMU(8);
    pigeon.configFactoryDefault();
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);

    // Set to aiming Mode
    this.setAimingMode();
   
    // For Path following
    this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));
  }

  public double getDriveInvert() {
    return driveInvert;
  }

  public void setDriveInvert(double driveInvert) {
    this.driveInvert = driveInvert;
  }

  public void initDefaultCommand() {
  }

  @Override
  public void periodic() {

    // Update the distance
    this.leftDistanceTraveled = this.leftEncoder.getDistance();
    SmartDashboard.putNumber("Left Distance", this.leftDistanceTraveled);

    this.rightDistanceTraveled = this.rightEncoder.getDistance();
    SmartDashboard.putNumber("Right Distance", this.rightDistanceTraveled);

    // update the turn angle
    // double[] xyz_dps = new double[3];
    double[] ypr_deg = new double[3];
    // short[] ba_xyz_acc = new short[3];

    // Query the 9DOF sensor
    this.pigeon.getYawPitchRoll(ypr_deg);
    this.curX = ypr_deg[0];
    // this.curY = ypr_deg[1];
    // this.curZ = ypr_deg[2];

    SmartDashboard.putNumber("Drive Compass", this.pigeon.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Drive Yaw", this.curX);

    // SmartDashboard.putNumber("Drive Pitch", this.curY);
    // SmartDashboard.putNumber("Drive Roll", this.curZ);
    // SmartDashboard.putNumber("X Accelerometer", this.curZ*100);

    // Update the odometry in the periodic block
    this.odometry.update(Rotation2d.fromDegrees(this.getHeading()), 
                                           this.leftEncoder.getDistance(),
                                           this.rightEncoder.getDistance()
                    );

    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // this.horizontalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    this.verticalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    this.getDistanceToTarget();

  }

  public void arcade(double speed, double direction) {
    /* Takes parameters and sets direction */

    // System.out.format("speed=%d--direction=%d", speed, direction);

    this.differentialDrive.arcadeDrive(speed * speedLimit * driveInvert, direction * turnLimit);
  }

  // Change robot speed limit. Based on buttons 2 and 3 in RobotContainer
  public void setLimitNorm() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: Speed limit norm");
    }
    this.speedLimit = DriveConstants.kDriveNorm;
    this.turnLimit = DriveConstants.kTurnNorm;
  }

  public void setLimitFast() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: Speed limit fast");
    }

    this.speedLimit = DriveConstants.kDriveFast;
    this.turnLimit = DriveConstants.kTurnFast;
  }

  public void setLimitSlow() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: slow");
    }
    this.speedLimit = DriveConstants.kDriveSlow;
    this.turnLimit = DriveConstants.kTurnSlow;
  }

  public void setIntakeMode() {
    this.driveInvert = -1.0;
    this.setLimeLightOff();
    this.setLimeLightNormalMode();
    SmartDashboard.putString("Mode", "Intake");
    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: Intake");
    }
  }

  public void setAimingMode() {
    this.driveInvert = 1.0;
    this.setLimeLightOn();
    this.setLimeLightDetectionMode();
    SmartDashboard.putString("Mode", "Aiming");
    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: Aiming");
    }
  }

  public double getDriveMode() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: get drive mode");
    }
    return this.driveInvert;
  }
  //  In the future replace with a getDistLeft() and getDistRight()
  public double getDistanceTraveled() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: get distance travelled");
    }
    return (this.leftDistanceTraveled + this.rightDistanceTraveled) / 2;
  }

  public double getCurrentHeading() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: get heading");
    }
      return this.curX;

  }

   /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(this.curX, 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.leftEncoder.getRate(), this.rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    this.odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

    /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: reset encoders");
    }
    this.leftEncoder.reset();
    this.rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (this.leftEncoder.getDistance() + this.rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    this.left.setVoltage(leftVolts);
    this.right.setVoltage(-rightVolts);
    this.differentialDrive.feed();
  }



  public double getDistanceToTarget() {

    if ( DriveConstants.kDebugDrive > 0 ) {
      System.out.println("Drive: get distance to target");
    }

    if ( this.targetAquired == 0.0) {
        return 0.0;
    }

    double actualAngle = this.verticalOffset + Constants.DriveConstants.kCameraAngle;
    double actualHeightOffset = Constants.DriveConstants.kTargetHeight - Constants.DriveConstants.kCameraHeight;
    double distance = actualHeightOffset / Math.tan(Math.toRadians(actualAngle));

    SmartDashboard.putNumber("Actual Angle", actualAngle);

    SmartDashboard.putNumber("Distance (feet)", distance * 39.37 / 12);

    return distance;
      
  }

   public void setLimeLightNormalMode() {

    if ( DriveConstants.kDebugLimeLight > 0 ) {
      System.out.println("Drive Limelight: normal mode");
    }
    this.setLimeLightOff();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

   }

   public void setLimeLightDetectionMode() {

    if ( DriveConstants.kDebugLimeLight > 0 ) {
      System.out.println("Drive Limelight: detection mode");
    }
    this.setLimeLightOn();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

   }

   
   public void setLimeLightOn() {

    if ( DriveConstants.kDebugLimeLight > 0 ) {
      System.out.println("Drive Limelight: led on");
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

   }

   public void setLimeLightOff() {

    if ( DriveConstants.kDebugLimeLight > 0 ) {
      System.out.println("Drive Limelight: led off");
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

   }

}
