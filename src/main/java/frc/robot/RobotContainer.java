// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.TunerConstatns;
import frc.robot.Subsystems.Drivebase.AlignCommand;
// import frc.robot.Subsystems.Drivebase.AlignCommand;
import frc.robot.Subsystems.Drivebase.AlignReefCommand;
import frc.robot.Subsystems.Drivebase.AlignReefCommandTake2;
import frc.robot.Subsystems.Drivebase.Drivebase;
import frc.robot.Subsystems.Drivebase.RotateReefCommand;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.Subsystems.PoseEstimator.PoseEstimator;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.misc.Calc;
import frc.robot.Subsystems.Field;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.RotationsPerSecond;

// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

public class RobotContainer {


  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1); 
  private final Systems systems = new Systems();
  private static final Vision vision = Systems.getVision();
  private static final Drivebase drivebase = Systems.getDrivebase();
  public Field field = new Field();

  private PoseEstimator poseEstimator = new PoseEstimator(() -> drivebase.getRotation3d().toRotation2d(), () -> drivebase.getState().ModulePositions);
  

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
        poseEstimator.setAlliance(Field.isBlue() ? Alliance.Blue : Alliance.Red);

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

  public double deadzone(double num) {
		if (Math.abs(num) > ControllerConstants.deadzone) {
			double w = 1.0 / ( 1.0 - ControllerConstants.deadzone);
			double b = w * ControllerConstants.deadzone;
			return (w * Math.abs(num) - b) * (num / Math.abs(num));
		} else {
			return 0;
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
    vision.periodic();
    poseEstimator.periodic();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
  }
  
  

  private Trigger commandTask = driver.a();

  private void configureBindings() {

      d_resetGyro.onTrue(drivebase.zeroGyro());
      // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivebase.setDefaultCommand(
        drivebase.applyRequest(
						() -> 
              drivebase.getDriveFieldCentric()
              // drivebase.getDriveRobotCentric()
								.withVelocityX(deadzone(-driver.getLeftY())
										* Constants.TunerConstatns.kSpeedAt12Volts.in(Units.MetersPerSecond))
								.withVelocityY(deadzone(-driver.getLeftX())
										* Constants.TunerConstatns.kSpeedAt12Volts.in(Units.MetersPerSecond))
								.withRotationalRate(
										deadzone(-driver.getRightX())
												* DrivebaseConstants.MaxAngularRate.in(Units.RadiansPerSecond)))
						.withName("Swerve Default Command"));
// driver.b().onTrue(drivebase.driveRobotCentric(new ChassisSpeeds(2,2,0)).withName("slam head in zzzz"));   
          // driver.x().onTrue(new AlignCommand(false).withName("Align Command"));
          // driver.b().onTrue(new AlignCommand(true).withName("Align Reef Command"));
          driver.a().onTrue(drivebase.setPose2dPositionCommand(vision.calculateRobotPositionFromTag()).withName("setting pose position"));
          // // driver.x().onTrue(new AlignReefCommandTake2(false).withName("Align Reef Command 2"));
          // // commandTask.onTrue(poseEstimator.testcommand());
          // driver.a().onTrue(
            
          //   drivebase.applyRequest(
          //       () -> drivebase.getDriverFieldCentricFacingAngle()
          //           .withVelocityX(0.0)  // Set X velocity (forward/backward speed in m/s)
          //           .withVelocityY(0.0)  // Set Y velocity (sideways speed in m/s)
          //           .withTargetDirection(drivebase.getAprilTagRotation().rotateBy(new Rotation2d(Math.PI/2))) // Set target direction (90 degrees, facing along Y-axis)
          //           .withTargetRateFeedforward(2.0) // Set rotational feedforward in rad/s
          //           .withMaxAbsRotationalRate(5.0)
          //           .withHeadingPID(1,0, 0.01)
          //           )
          //       .raceWith(new WaitUntilCommand(() -> test_command()))
          // ); //TODO: FIX THIS
          // // driver.x().onTrue(new RotateReefCommand().withName("Rotation Reef Command"));
          // // driver.x().onTrue(drivebase.faceAprilTag());
          // // driver.a().onTrue(field.getAprilTagPose3dCommand());
          // driver.a().onTrue(poseEstimator);
          // // driver.rightBumper().onTrue(drivebase.randomTst());

          // driver.a().onTrue(
          //   drivebase.applyRequest(
          //       () -> drivebase.getDriveRobotCentric()
          //           .withVelocityX(getXValue())  // Set X velocity (forward/backward speed in m/s)
          //           .withVelocityY(getYValue())  // Set Y velocity (sideways speed in m/s)
          // ));


          // driver.a().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(10), 1), true));
          driver.b().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(18), 0), true).withName("Running Path"));
          // driver.x().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(10), 2), true));
          // driver.rightBumper().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(9), 2), true));

          // driver.leftBumper().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(10), 0), true));
          // driver.rightTrigger().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(9), 0), true));
          // driver.leftTrigger().onTrue(new DriveToPoseCommand(drivebase, () -> drivebase.getRobotPose(), convert3DPoseTo2D(field.getAprilTagPose3d(3), 1), true));

          

    } 

    public boolean test_command(){
        double currentHeading = drivebase.getRobotPose().getRotation().getRadians();
        double setPoint = drivebase.getDriverFieldCentricFacingAngle().HeadingController.getSetpoint();
        boolean atSetpoint = drivebase.getDriverFieldCentricFacingAngle().HeadingController.atSetpoint();
        System.out.println(currentHeading);
        System.out.println(setPoint);
        System.out.println(atSetpoint);
        System.out.println(drivebase.getDriverFieldCentricFacingAngle().HeadingController.getPositionError());
        // System.out.println(drivebase.getDriverFieldCentricFacingAngle().HeadingController.getLastAppliedOutput());
        drivebase.getDriverFieldCentricFacingAngle().HeadingController.setTolerance(0.1);
        drivebase.getDriverFieldCentricFacingAngle().HeadingController.disableContinuousInput();


        // Check if the current heading is within the tolerance of the target heading
        return Calc.approxEquals(currentHeading, setPoint, 0.5) && atSetpoint;
        // return drivebase.getDriverFieldCentricFacingAngle().HeadingController.atSetpoint();
    }
 
    public Pose2d convert3DPoseTo2D(Pose3d pose, int alignReef) {
      // alignReef
      //    0 = Center
      //    1 = Right
      //    2 = Left
      pose = translatPose3d(pose, alignReef);

      // Translation (x, y) stays the same
      double translationX = pose.getX();
      double translationY = pose.getY();
      Rotation2d rotation = pose.getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));

      // Rotation2d rotation = field.getAprilTagPose3d(16).getRotation().toRotation2d();
      
      // Return the resulting 2D pose
      return new Pose2d(new Translation2d(translationX, translationY), rotation);
    }

    public static Pose3d translatPose3d(Pose3d pose, int reefSelect){
      // reefSelect:
      //    0 = Center
      //    1 = Right
      //    2 = Left

      double distanceAway = Constants.AutonConstants.reefAproach;
      double distanceSide = 0;
      switch(reefSelect){
        case 1:
          distanceSide = Constants.AutonConstants.rightReefoffset;
          break;
        case 2:
          distanceSide = Constants.AutonConstants.leftReefoffset;
          break;
      }

      Translation3d tagTranslation = pose.getTranslation();
      Rotation3d tagRotation = pose.getRotation();

      // Moving forward is along the tag's forward direction (along the tag's facing direction)
      double tagYaw = tagRotation.getZ();  // The yaw is the rotation around the z-axis (2D rotation)

       // Calculate the new position 2 meters to the right of the tag
       double sideX = Math.cos(tagYaw + Math.PI / 2) * distanceSide;  // Move 2 meters to the right in x direction
       double sideY = Math.sin(tagYaw + Math.PI / 2) * distanceSide;  // Move 2 meters to the right in y direction

       // Calculate the new position 1 meter away from the tag (moving along the tag's facing direction)
       double awayX = Math.cos(tagYaw) * distanceAway;  // Move 1 meter in front of the tag in x direction
       double awayY = Math.sin(tagYaw) * distanceAway;  // Move 1 meter in front of the tag in y direction

       // Add both translations (right and away)
       double newX = tagTranslation.getX() + sideX + awayX;
       double newY = tagTranslation.getY() + sideY + awayY;
       double newZ = tagTranslation.getZ();  // z stays the same

      // Step 4: Create the new Pose3d for the robot's position and rotation
      return new Pose3d(newX, newY, newZ, tagRotation);

    }

    // public double getXValue(){

    // }

    // public double getYValue(){

    // }
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