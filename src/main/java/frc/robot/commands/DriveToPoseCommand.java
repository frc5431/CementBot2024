package frc.robot.commands;

import static frc.robot.Constants.AutonConstants.THETA_kD;
import static frc.robot.Constants.AutonConstants.THETA_kI;
import static frc.robot.Constants.AutonConstants.THETA_kP;
import static frc.robot.Constants.AutonConstants.X_kD;
import static frc.robot.Constants.AutonConstants.X_kI;
import static frc.robot.Constants.AutonConstants.X_kP;
import static frc.robot.Constants.AutonConstants.Y_kD;
import static frc.robot.Constants.AutonConstants.Y_kI;
import static frc.robot.Constants.AutonConstants.Y_kP;
import static frc.robot.Constants.DrivebaseConstants.AutonMaxAngularRate;
import static frc.robot.Constants.DrivebaseConstants.AutonMaxVelocity;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Systems;
import frc.robot.Subsystems.Drivebase.Drivebase;

/**
 * Command to drive to a pose.
 */

public class DriveToPoseCommand extends Command {
  
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    25, // TODO: Convert this to double - AutonMaxVelocity * 0.5,
    5.0); // Convert this to double as well - nAutonMaxVelocity);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    0.2, // TODO: Convert these to double - AutonMaxAngularRate * 0.4,
    0.5); //AutonMaxAngularRate);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private Drivebase drivebase = Systems.getDrivebase();
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean useAllianceColor;

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        // LEDSubsystem ledSubsystem,
        boolean useAllianceColor) {
    this(drivebase, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
  }

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        // LEDSubsystem ledSubsystem,
        boolean useAllianceColor) {
    this.drivebase = drivebase;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivebase);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    
    if (useAllianceColor) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivebase.driveRobotCentric(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.stopRobotCentric();
  }

}