/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
// import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

// import java.util.function.BooleanSupplier;

// import javax.swing.text.DefaultEditorKit.BeepAction;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.ColorWheel;

// import frc.robot.Constants;
import frc.robot.Constants.BeefCakeConstants;

import com.ctre.phoenix.sensors.PigeonIMU;

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

  private PigeonIMU pigeon;
  private double curX;
  private double curY;
  private double curZ;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  public String targetColor;
  private String currentColor;


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

    // Initialize the Pigeon 9DOF
    this.pigeon = new PigeonIMU(9);
    this.pigeon.configFactoryDefault();   
    
    this.colorMatcher.addColorMatch(BeefCakeConstants.kBlueTarget);
    this.colorMatcher.addColorMatch(BeefCakeConstants.kGreenTarget);
    this.colorMatcher.addColorMatch(BeefCakeConstants.kRedTarget);
    this.colorMatcher.addColorMatch(BeefCakeConstants.kYellowTarget);    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = colorSensor.getColor();

     //  Run the color match algorithm on our detected color
    // String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    
    if ( gameData != null ) {
      switch (gameData.charAt(0)) {
        case 'G':
          this.targetColor = "GREEN";
          break;
        case 'Y':
          this.targetColor = "YELLOW";
          break;
        case 'B':
          this.targetColor = "BLUE";
          break;
        case 'R':
          this.targetColor = "RED";
          break;
      }
    } else {
        this.targetColor = "UNKNOWN";
    }

    SmartDashboard.putString("Target Color", this.targetColor);

    if (match.color == BeefCakeConstants.kBlueTarget) {
      this.currentColor = "Blue";
    } else if (match.color == BeefCakeConstants.kRedTarget) {
      this.currentColor = "Red";
    } else if (match.color == BeefCakeConstants.kGreenTarget) {
      this.currentColor = "Green";
    } else if (match.color == BeefCakeConstants.kYellowTarget) {
      this.currentColor = "Yellow";
    } else {
      this.currentColor = "Unknown";
    }

    //  Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", this.currentColor);    

    // update the turn angle
    // double[] xyz_dps = new double[3];
    double[] ypr_deg = new double[3];
    // short[] ba_xyz_acc = new short[3];

    // Query the 9DOF sensor
    this.pigeon.getYawPitchRoll(ypr_deg);
    this.curX = ypr_deg[0];
    this.curY = ypr_deg[1];
    this.curZ = ypr_deg[2];

    SmartDashboard.putNumber("Beef Compass", this.pigeon.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Beef Yaw", this.curX);
    SmartDashboard.putNumber("Beef Pitch", this.curY);
    SmartDashboard.putNumber("Beef Roll", this.curZ);
    SmartDashboard.putNumber("Beef X Accelerometer", this.curZ*100);
  }

  //  Turns the feeder On and Off
  public void feederOn() {

    if ( BeefCakeConstants.kDebugBeefCakeFeeder > 0) {
      System.out.println("Feeder: Feeder is active");
    }

    feed.set(BeefCakeConstants.kFeederSpeed);
  }

  public void feederOff() {

    if ( BeefCakeConstants.kDebugBeefCakeFeeder > 0 ) {
      System.out.println("Feeder: Feeder is not active");
    }

    feed.stopMotor();
  }

  //  Turns the launcher wheels On and Off
  public void launcherOn() {
    if ( BeefCakeConstants.kDebugBeefCakeLauncher > 0 ) {
      System.out.println("Launcher: launcherOn is active");
    }

    launcher.set(BeefCakeConstants.kLauncherSpeed);
  }

  public void launcherOff() {

    if ( BeefCakeConstants.kDebugBeefCakeLauncher > 0  ) {
      System.out.println("Launcher: launcherOff is active");
    }

    launcher.stopMotor();
  }

  //  Adjusts the launcher Up and Down (using the angle of the co-pilot joystick)
  public void angleJoystick(double speed) {
    getCurrentAngle();
    angleMotors.set(speed);
  }

  //  Flip out Launcher
  public void intakeFlipOut() {
    angleMotors.set(-0.8);
    Timer.delay(0.5);
    angleMotors.stopMotor();
  }

  //  Turns the intake on and off
  public void intakeOn() {
    if ( BeefCakeConstants.kDebugBeefCakeIntake > 0) {
      System.out.println("Intake: intake is on");
    }

    intake.set(BeefCakeConstants.kIntake);
  }

  public void intakeReverse() {
    if (BeefCakeConstants.kDebugBeefCakeIntake > 0) {
      System.out.println("Intake: intake is reverse");
    }

    intake.set(-1.0*BeefCakeConstants.kIntake);
  }


  public void intakeOff() {
    if ( BeefCakeConstants.kDebugBeefCakeIntake > 0) {
      System.out.println("Intake: intake is off");
    }

    intake.stopMotor();
  }

  public void climbUp() {
    if ( BeefCakeConstants.kDebugBeefCakeClimber > 0 ) {
      System.out.println("Climber: up");
    }

    climber.set(0.9);
  }

  public void climbDown() {
    if ( BeefCakeConstants.kDebugBeefCakeClimber > 0) {
      System.out.println("Climber: climber is active");
    }

    climber.set(-0.9);
  }

  public void climbOff() {
    if ( BeefCakeConstants.kDebugBeefCakeClimber > 0 ) {
      System.out.println("CLimber: climber is off");
    }

    climber.stopMotor();
  }

  public void tare() {

    if ( BeefCakeConstants.kDebugBeefCakeAngle > 0 ) {
      System.out.println("Angle: Tare");
    }

    this.pigeon.setYaw(0.0);
    this.pigeon.setFusedHeading(0.0);

  }

  public double getCurrentAngle() {
    
    if ( BeefCakeConstants.kDebugBeefCakeAngle > 0 ) {
      System.out.println("Angle: getting value");
    }

    double currentAngle = this.curZ - BeefCakeConstants.kStartingAngle;
    SmartDashboard.putNumber("BeefCake Angle", currentAngle);

    return currentAngle;
  }

  public void moveAngle(double speed) { 

    if ( BeefCakeConstants.kDebugBeefCakeAngle > 0 ) {
      System.out.println("Angle: move at " + speed );
    }

    this.getCurrentAngle();
    angleMotors.set(speed);
  }

  public void stopAngle() { 

    if ( BeefCakeConstants.kDebugBeefCakeAngle > 0 ) {
      System.out.println("Angle: Stop");
    }
    angleMotors.stopMotor();
  }

  public Integer colorOffset() {

    return ColorWheel.valueOf(this.currentColor).ordinal() - ColorWheel.valueOf(this.targetColor).ordinal();

  }
}
