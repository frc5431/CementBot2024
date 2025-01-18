package frc.robot;



import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.TunerConstatns;
import frc.team5431.titan.core.leds.Blinkin;
import frc.robot.subsystems.Drivebase;

public class Systems {
  public static Systems instance;

  private Blinkin blinkin;

  private MotorType brushless =  MotorType.kBrushless;
  public Drivebase pheonixdrivebase;

  public Systems() {

    blinkin = new Blinkin(0);
    
     pheonixdrivebase = TunerConstatns.createDrivetrain();

    instance = this;

    // LasaVision.getInstance().setPoseSupplier(() -> pheonixdrivebase.getPose());

  }

  public Drivebase getDrivebase() {
    return pheonixdrivebase;
  }
}
