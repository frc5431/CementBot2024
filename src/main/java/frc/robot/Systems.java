package frc.robot;



import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.Drivebase.Drivebase;
import frc.robot.Subsystems.Limelight.Vision;
import frc.team5431.titan.core.joysticks.TitanController;
import frc.team5431.titan.core.leds.Blinkin;

public class Systems {
  private static TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
    public static TitanController getDriver() {
    return driver;
  }

    private static TitanController operator = new TitanController(ControllerConstants.operatorPort,
            ControllerConstants.deadzone);
  public static TitanController getOperator() {
      return operator;
    }

  public static Systems instance;

      private static Vision vision;

  public static Vision getVision() {
        return vision;
      }

  private Blinkin blinkin;

  private MotorType brushless =  MotorType.kBrushless;
  private static Drivebase drivebase = new Drivebase(
        Constants.TunerConstatns.DrivetrainConstants,
        Constants.TunerConstatns.FrontLeft, Constants.TunerConstatns.FrontRight,
        Constants.TunerConstatns.BackLeft, Constants.TunerConstatns.BackRight);

  public static Drivebase getDrivebase() {
    return drivebase;
  }

  public Systems() {

    blinkin = new Blinkin(0);
    

    instance = this;

    // LasaVision.getInstance().setPoseSupplier(() -> pheonixdrivebase.getPose());

  }

}
