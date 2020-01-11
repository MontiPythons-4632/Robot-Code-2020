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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class BeefCake extends SubsystemBase {
  /**
   * Creates a new BeefCake.
   */
  private WPI_VictorSPX angleTop;
  private WPI_VictorSPX angleBottom;
  private Spark launcherLeft;
  private Spark launcherRight;
  private Spark feed;
  private SpeedControllerGroup angle;
  private SpeedControllerGroup launcher;

  public BeefCake() {
    angleTop = new WPI_VictorSPX(6);
    angleBottom = new WPI_VictorSPX(7);
    angle = new SpeedControllerGroup(angleTop, angleBottom);
    
    launcherLeft = new Spark(1);
    addChild("Launcher Left", launcherLeft);
    launcherLeft.setInverted(false);

    launcherRight = new Spark(2);
    addChild("Launcher Right", launcherRight);
    launcherRight.setInverted(true);

    launcher = new SpeedControllerGroup(launcherRight, launcherLeft);
    addChild("Launcher", launcher);

    feed = new Spark(0);
    addChild("Feed", feed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void feedOn(double speed) {
    feed.setSpeed(speed);
  }

  public void feedOff() {
    feed.stopMotor();
  }

  public void launcherOn(double volts) {
    launcher.setVoltage(volts);
  }

  public void launcherOff() {
    launcher.stopMotor();
  }

  public void adjustAngle(double volts) {
    angle.setVoltage(volts);
  }

}
