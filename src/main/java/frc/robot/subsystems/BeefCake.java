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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.Constants;
import frc.robot.Constants.*;

public class BeefCake extends SubsystemBase {
  /**
   * Creates a new BeefCake.
   */
  private final Spark angleMotors;
  private final WPI_VictorSPX feed;
  private final Spark climber1;
  private final Spark climber2;
  private final SpeedControllerGroup climber;
  private final WPI_VictorSPX launcherLeft;
  private final WPI_VictorSPX launcherRight;
  private final Spark intake;
  private final SpeedControllerGroup launcher;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();


  public BeefCake() {
    angleMotors = new Spark(2);
    addChild("angleMotors", angleMotors);

    feed = new WPI_VictorSPX(8);
    addChild("tower", feed);
    feed.setInverted(true);

    climber1 = new Spark(0);
    feed.setInverted(false);

    climber2 = new Spark(3);
    feed.setInverted(true);

    climber = new SpeedControllerGroup(climber1, climber2);
    addChild("Climber", climber);

    launcherLeft = new WPI_VictorSPX(5);
    addChild("Launcher Right", launcherLeft);
    launcherLeft.setInverted(false);

    launcherRight = new WPI_VictorSPX(6);
    addChild("Launcher Right", launcherRight);
    launcherRight.setInverted(true);

    launcher = new SpeedControllerGroup(launcherLeft, launcherRight);
    addChild("Launcher", launcher);

    intake = new Spark(1);
    addChild("Intake", intake);
    intake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = colorSensor.getColor();

     //  Run the color match algorithm on our detected color
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == Constants.BeefCakeConstants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.BeefCakeConstants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.BeefCakeConstants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.BeefCakeConstants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);    
  }

  //  Turns the feeder On and Off
  public void feederOn() {

    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("Feeder is active");
    }

    feed.set(BeefCakeConstants.kFeederSpeed);
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
  
  // public void adjustAngleUp() {

  //   if ( BeefCakeConstants.DEBUG ) {
  //     System.out.println("Adjusting angle up");
  //   }

  //   angleMotors.set(BeefCakeConstants.kAngleSpeed*0.6);
  // }

  // public void adjustAngleDown() {

  //   if ( BeefCakeConstants.DEBUG ) {
  //     System.out.println("Adjusting angle down");
  //   }

  //   angleMotors.set(BeefCakeConstants.kAngleSpeed * -1.0);
  // }

  // public void stopAngle() {
  //   angleMotors.stopMotor();
  // }

  //  Adjusts the launcher Up and Down (using the angle of the co-pilot joystick)
  public void angleJoystick(double speed) {
    angleMotors.set(speed);
  }

  //  Turns the intake on and off
  public void intakeOn() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("intake is active");
    }

    intake.set(BeefCakeConstants.kIntake);
  }

  public void intakeReverse() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("intake is active");
    }

    intake.set(-1.0*BeefCakeConstants.kIntake);
  }


  public void intakeOff() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("intake is not active");
    }

    intake.stopMotor();
  }

  public void climbUp() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("climber is active");
    }

    climber.set(0.9);
  }

  public void climbDown() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("climber is active");
    }

    climber.set(-0.9);
  }

  public void climbOff() {
    if ( BeefCakeConstants.DEBUG ) {
      System.out.println("climber is active");
    }

    climber.stopMotor();
  }
}
