/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

import java.util.function.BooleanSupplier;

import javax.swing.text.DefaultEditorKit.BeepAction;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import frc.robot.Constants.BeefCakeConstants;

public class BeefCake extends SubsystemBase {
  /**
   * Creates a new BeefCake.
   */
  private final WPI_VictorSPX angleTop;
  private final WPI_VictorSPX angleBottom;
  private final Spark launcherLeft;
  private final Spark launcherRight;
  private final Spark feed;
  private final SpeedControllerGroup angle;
  private final SpeedControllerGroup launcher;

  public BeefCake() {
    angleTop = new WPI_VictorSPX(6);
    angleBottom = new WPI_VictorSPX(7);
    angle = new SpeedControllerGroup(angleTop, angleBottom);

    launcherLeft = new Spark(1);
    addChild("Launcher Left", launcherLeft);
    launcherLeft.setInverted(true);

    launcherRight = new Spark(2);
    addChild("Launcher Right", launcherRight);
    launcherRight.setInverted(false);

    launcher = new SpeedControllerGroup(launcherRight, launcherLeft);
    addChild("Launcher", launcher);

    feed = new Spark(0);
    addChild("Feed", feed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("----------------------------------memes lol");
  }

  //  Turns the feeder On and Off
  public void feederOn() {

    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("Feeder is active");
    }

    feed.setSpeed(BeefCakeConstants.kFeederSpeed);
  }

  public void feederOff() {

    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("Feeder is not active");
    }

    feed.stopMotor();
  }

  // public boolean isFeederOn() {

  //   if ( this.feed.getSpeed() > 0.0 ) {
  //       return true;
  //   } else {
  //       return false;
  //   }
  // }

  // public BooleanSupplier isLauncherOn() {

  //   if ( this.feed.getSpeed() > 0.0 ) {
  //       return () -> true;
  //   } else {
  //       return () -> false;
  //   }
  // }

  //  Turns the launcher wheels On and Off
  public void launcherOn() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("launcherOn is active");
    }

    launcher.set(BeefCakeConstants.kLauncherSpeed);
  }

  public void launcherOff() {

    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("launcherOff is active");
    }

    launcher.stopMotor();
  }

  //  Adjusts the angle of the launcher Up and Down, and Stops it (using buttons)
  // public void adjustAngleUp() {

  //   if ( BeefCakeConstants.DEBUG ) {
  //     System.out.println("adjustAngleUp is active");
  //   }

  //   angle.set(BeefCakeConstants.kAngleSpeed);
  // }

  // public void adjustAngleDown() {

  //   if ( BeefCakeConstants.DEBUG ) {
  //     System.out.println("adjustAngleDown is active");
  //   }

  //   angle.set(BeefCakeConstants.kAngleSpeed * -1.0);
  // }

  // public void stopAngle() {
  //   angle.stopMotor();
  // }

  //  Adjusts the launcher Up and Down (using the angle of the co-pilot joystick)
  public void angleJoystick(double speed) {
    angle.set(speed);
  }

  //  Turns the intake on and off
  public void intakeOn() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("intake is active");
    }

    // feed.setSpeed(BeefCakeConstants.kFeederSpeed);
  }

  public void intakeOff() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("intake is not active");
    }

    // feed.setSpeed(BeefCakeConstants.kFeederSpeed);
  }

}
