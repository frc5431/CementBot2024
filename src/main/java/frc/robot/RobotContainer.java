// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TunerConstatns;
import frc.robot.subsystems.Drivebase;
import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1); 
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

  // private final Blinkin blinkin = systems.getBlinkin();
  private final AutonMagic autonMagic;

  TitanFieldCentricFacingAngle facingRequest = new TitanFieldCentricFacingAngle();

  SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private SwerveRequest.RobotCentric driveRo = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  // Control Schemes
  Trigger d_robotOriented = driver.rightBumper();
  Trigger d_resetGyro = driver.y();

  public RobotContainer() {
    autonMagic = new AutonMagic();


    // drivebase.seedField Relative();
    facingRequest.withPID(new PIDController(6, 0.01, 0.008));
    facingRequest.withDampening(new WeightedAverageController(45));
    facingRequest.gyro = drivebase.getGyro();
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
  
  private Command lockToAngleCommand(double redAngle, double blueAngle) {
    return drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      return facingRequest
          .withVelocityX(
              modifyAxis(y2)
                  * TunerConstatns.kSpeedAt12VoltsMps)
          .withHeading(edu.wpi.first.math.util.Units.degreesToRadians((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? blueAngle : redAngle))
          .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps);
    }).until(() -> Math.abs(driver.getRawAxis(4)) > 0.15);
  }

  private void configureBindings() {
    drivebase.setDefaultCommand( // Drivetrain will execute this command periodically
        drivebase.applyRequest(() -> {
         double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);


          return driveFC
              .withVelocityX(
                  modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                      * TunerConstatns.kSpeedAt12VoltsMps)
              .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps)
              .withRotationalRate(
                  modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }));

    driver.povUp().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(180, 180));

    driver.povUpRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(225, 225));

    driver.povRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(270, 270));

    driver.povDownRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(315, 315));

    driver.povDown().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(0, 0));

    driver.povDownLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(45, 45));

    driver.povLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(90, 90));

    driver.povUpLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(135, 135));

    d_robotOriented.whileTrue(drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      return driveRo
          .withVelocityX(
              modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                  * TunerConstatns.kSpeedAt12VoltsMps)
          .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps)
          .withRotationalRate(
              modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }));
   
    }
 

    // blinkin.setDefaultCommand(new InstantCommand(() -> blinkin.set(BlinkinPattern.CP1_2_TWINKLES), blinkin));    

    // d_rightClimber.whileTrue(rightClimber.increment(0.8).repeatedly());
    // d_leftClimber.whileTrue(leftClimber.increment(0.8).repeatedly());


    // o_strobeLights.whileTrue(new BlinkinStrobeCommand(systems.getBlinkin(), BlinkinPattern.ORANGE));
    // operator.start().whileTrue(shooter.runShooterCommand(ShooterModes.AmpShot));


  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
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