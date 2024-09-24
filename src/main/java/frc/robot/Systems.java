package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
    
    pheonixdrivebase = new Drivebase(TunerConstatns.DrivetrainConstants, TunerConstatns.FrontLeft, TunerConstatns.FrontRight, TunerConstatns.BackLeft, TunerConstatns.BackRight);

    instance = this;

    // LasaVision.getInstance().setPoseSupplier(() -> pheonixdrivebase.getPose());

  }

  public Drivebase getDrivebase() {
    return pheonixdrivebase;
  }
}
