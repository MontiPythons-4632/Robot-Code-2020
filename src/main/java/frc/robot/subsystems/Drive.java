/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;

import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  //Creates a new Drive.

  private WPI_TalonSRX leftFront;
  private WPI_VictorSPX leftBack;
  private SpeedControllerGroup left;
  private WPI_TalonSRX rightFront;
  private WPI_VictorSPX rightBack;
  private SpeedControllerGroup right;
  private DifferentialDrive differentialDrive;
  private double speedLimit;
  private boolean aimingMode;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private double leftDistanceTraveled;
  private double rightDistanceTraveled;
  private AnalogGyro analogGyro1;

  // Move this to constants%
  private static final double cpr = 214; //if am-3314a
  private static final double whd = 6; // for 6 inch wheel


  public Drive() {
    leftFront = new WPI_TalonSRX(1);
    leftBack = new WPI_VictorSPX(2);
    left = new SpeedControllerGroup(leftFront, leftBack);
    left.setInverted(this.aimingMode);
    addChild("Left",left);
    
    rightFront = new WPI_TalonSRX(3);
    rightBack = new WPI_VictorSPX(4);
    right = new SpeedControllerGroup(rightFront, rightBack);
    right.setInverted(this.aimingMode);
    addChild("Right",right);
    
    differentialDrive = new DifferentialDrive(left, right);
    addChild("Differential Drive",differentialDrive);
    differentialDrive.setSafetyEnabled(false);
    differentialDrive.setExpiration(0.3);
    differentialDrive.setMaxOutput(1.0);

    this.speedLimit = DriveConstants.kDriveNorm;

    // factory default values
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    //[4] adjust sensor phase so sensor moves positive when Talon LEDs are green
    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);
    rightBack.setSensorPhase(true);
    leftBack.setSensorPhase(true);

    // Set up encoder
    leftEncoder = new Encoder(6,7);
    leftEncoder.reset();
    leftEncoder.setDistancePerPulse(Math.PI*whd/cpr); //distance per pulse is pi* (wheel diameter / counts per revolution)
    leftEncoder.setReverseDirection(!this.aimingMode);

    rightEncoder = new Encoder(8,9);
    rightEncoder.reset();
    rightEncoder.setDistancePerPulse(Math.PI*whd/cpr); //distance per pulse is pi* (wheel diameter / counts per revolution)
    rightEncoder.setReverseDirection(this.aimingMode);

    // Setup Up Pidgeon
    analogGyro1 = new AnalogGyro(0);

    addChild("AnalogGyro 1",analogGyro1);

    analogGyro1.setSensitivity(0.007);
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

  }

  public void arcade(double speed, double direction) {
    /* Takes parameters and sets direction */
  
    // System.out.format("speed=%d--direction=%d", speed, direction);

    this.differentialDrive.arcadeDrive(speed*this.speedLimit, direction*this.speedLimit);
  }

  //  Change robot speed limit. Based on buttons 2 and 3 in RobotContainer
  public void setLimitNorm() {
    this.speedLimit = DriveConstants.kDriveNorm;
  }
  
  public void setLimitFast() {
    this.speedLimit = DriveConstants.kDriveFast;
  }

  public void setLimitSlow() {
    this.speedLimit = DriveConstants.kDriveSlow;
  }

  public void reverseDrive() {
    this.aimingMode = !this.aimingMode;
  }


  //  In the future replace with a getDistLeft() and getDistRight()
  public double getDistanceTraveled() {
    return (this.leftDistanceTraveled + this.rightDistanceTraveled) / 2;
  }

}
