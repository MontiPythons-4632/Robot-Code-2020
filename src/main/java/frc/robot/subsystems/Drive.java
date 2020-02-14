/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.concurrent.atomic.DoubleAccumulator;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
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

  // Move this to constants
  // private static final double cpr = 214; // if am-3314a
  // private static final double whd = 6; // for 6 inch wheel

  //  Limelight variables
  private double targetAquired;
  private double horizontalOffset;
  private double verticalOffset;


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
    leftEncoder = new Encoder(6, 7);
    leftEncoder.reset();
    leftEncoder.setDistancePerPulse(Math.PI * DriveConstants.whd / DriveConstants.cpr); // distance per pulse is pi* (wheel diameter / counts per
                                                          // revolution)
    leftEncoder.setReverseDirection(true);

    rightEncoder = new Encoder(8, 9);
    rightEncoder.reset();
    rightEncoder.setDistancePerPulse(Math.PI * DriveConstants.whd / DriveConstants.cpr); // distance per pulse is pi* (wheel diameter / counts per
                                                           // revolution)
    rightEncoder.setReverseDirection(false);

    // Initialize the Pigeon 9DOF
    pigeon = new PigeonIMU(8);
    pigeon.configFactoryDefault();
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);

    // Set to aiming Mode
    this.setAimingMode();
    }

  public double getDriveInvert() {
    return driveInvert;
  }
    //memes lol
  public void setDriveInvert(double driveInvert) {
    this.driveInvert = driveInvert;
  }

  public void initDefaultCommand() {
  }

  @Override
  public void periodic() {

    // Update the distance
    this.leftDistanceTraveled = leftEncoder.getDistance();
    SmartDashboard.putNumber("Left Distance", this.leftDistanceTraveled);

    this.rightDistanceTraveled = rightEncoder.getDistance();
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

    SmartDashboard.putNumber("Compass", this.pigeon.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Yaw", this.curX);
    // SmartDashboard.putNumber("Pitch", this.curY);
    // SmartDashboard.putNumber("Roll", this.curZ);
    // SmartDashboard.putNumber("X Accelerometer", this.curZ*100);

    this.targetAquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    this.horizontalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
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
    this.speedLimit = DriveConstants.kDriveNorm;
    this.turnLimit = DriveConstants.kTurnNorm;
  }

  public void setLimitFast() {
    this.speedLimit = DriveConstants.kDriveFast;
    this.turnLimit = DriveConstants.kTurnFast;
  }

  public void setLimitSlow() {
    this.speedLimit = DriveConstants.kDriveSlow;
    this.turnLimit = DriveConstants.kTurnSlow;
  }

  public void setIntakeMode() {
    this.driveInvert = -1.0;
    SmartDashboard.putString("Mode", "Intake");
    System.out.println("Drive is inverted");
  }

  public void setAimingMode() {
    this.driveInvert = 1.0;
    SmartDashboard.putString("Mode", "Aiming");
    System.out.println("Drive is not inverted");
  }

  public double getDriveMode() {
    return this.driveInvert;
  }
  //  In the future replace with a getDistLeft() and getDistRight()
  public double getDistanceTraveled() {
    return (this.leftDistanceTraveled + this.rightDistanceTraveled) / 2;
  }

  public double getCurrentHeading() {

      return this.curX;

  }

  public double getDistanceToTarget() {

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

  // public void limeLightAlign() {
  //   this.setAimingMode();

  //   SmartDashboard.putString("Mode", "Aligning");
  //   if (this.targetAquired == 1) {
  //     SmartDashboard.putBoolean("Target Aquired", true);
  //   } else {
  //     SmartDashboard.putBoolean("Target Aquired", false);
  //   }

  //   System.out.println(this.horizontalOffset);
  //   this.TurnXDegrees(this.driveSubsystem, this.horizontalOffset);
  // }

}
