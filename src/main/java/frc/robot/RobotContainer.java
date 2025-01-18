// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.units.Units;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Drivebase;
import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.RotationsPerSecond;

// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class RobotContainer {

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1); 
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

   private double MaxSpeed = Constants.TunerConstatns.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); 

  // private final Blinkin blinkin = systems.getBlinkin();
  // private final AutonMagic autonMagic;

  TitanFieldCentricFacingAngle facingRequest = new TitanFieldCentricFacingAngle();

  SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private SwerveRequest.RobotCentric driveRo = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  // Control Schemes
  Trigger d_robotOriented = driver.rightBumper();
  Trigger d_resetGyro = driver.y();

  public RobotContainer() {
    // autonMagic = new AutonMagic();


    // drivebase.seedField Relative();
    configureBindings();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    // var alliance = DriverStation.getAlliance();
    // if(alliance.get() == DriverStation.Alliance.Red) {
    // value = -value;
    // }

    value = deadband(value, 0.15);

    // More sensitive at smaller speeds
    double newValue = Math.pow(value, 2);

    // Copy the sign to the new value
    newValue = Math.copySign(newValue, value);

    return newValue;
  }

  public void periodic() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
  }
  
  

  private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivebase.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivebase.applyRequest(() ->
              driveFC.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }
 

    // blinkin.setDefaultCommand(new InstantCommand(() -> blinkin.set(BlinkinPattern.CP1_2_TWINKLES), blinkin));    

    // d_rightClimber.whileTrue(rightClimber.increment(0.8).repeatedly());
    // d_leftClimber.whileTrue(leftClimber.increment(0.8).repeatedly());


    // o_strobeLights.whileTrue(new BlinkinStrobeCommand(systems.getBlinkin(), BlinkinPattern.ORANGE));
    // operator.start().whileTrue(shooter.runShooterCommand(ShooterModes.AmpShot));


  public Command getAutonomousCommand() {
  //   return autonMagic.procureAuton();
  return null;
  }

  public void onTeleop() {
    // amper.motor.getPIDController().setOutputRange(-1, 1);
    // amper.motor.burnFlash();
    // pivot.setpoint = Units.Radians.of(pivot.absoluteEncoder.getPosition());
    // amperPivot.setpoint = (Constants.AmperConstants.anglerConstants.minAngle);
    // rightClimber.relativeEncoder.setPosition(0);
    // leftClimber.relativeEncoder.setPosition(0);
    //rightClimber.setpoint = 0;
    //leftClimber.setpoint = 0;
  }

}