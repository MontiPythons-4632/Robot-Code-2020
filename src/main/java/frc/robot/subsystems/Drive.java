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

import frc.robot.Constants;

public class Drive extends SubsystemBase {
  //Creates a new Drive.

  private WPI_TalonSRX leftFront;
  private WPI_VictorSPX leftBack;
  private SpeedControllerGroup left;
  private WPI_TalonSRX rightFront;
  private WPI_VictorSPX rightBack;
  private SpeedControllerGroup right;
  private DifferentialDrive differentialDrive;

  public Drive() {
    leftFront = new WPI_TalonSRX(1);
    leftBack = new WPI_VictorSPX(2);
    left = new SpeedControllerGroup(leftFront, leftBack);
    addChild("Left",left);
    
    rightFront = new WPI_TalonSRX(3);
    rightBack = new WPI_VictorSPX(4);
    right = new SpeedControllerGroup(rightFront, rightBack);
    addChild("Right",right);
    
    differentialDrive = new DifferentialDrive(left, right);
    addChild("Differential Drive",differentialDrive);
    differentialDrive.setSafetyEnabled(false);
    differentialDrive.setExpiration(0.3);
    differentialDrive.setMaxOutput(1.0);

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
  }

  public void initDefaultCommand() {
  }

  @Override
  public void periodic() {
  }

  public void arcade(double speed, double direction) {
    /* Takes parameters and sets direction */
    this.differentialDrive.arcadeDrive(-1.0*speed, direction);
  }
}
