package frc.robot.commands;

import static frc.robot.Constants.AutonConstants.THETA_kD;
import static frc.robot.Constants.AutonConstants.THETA_kI;
import static frc.robot.Constants.AutonConstants.THETA_kP;
import static frc.robot.Constants.AutonConstants.D_kD;
import static frc.robot.Constants.AutonConstants.D_kI;
import static frc.robot.Constants.AutonConstants.D_kP;
import static frc.robot.Constants.DrivebaseConstants.AutonMaxAngularRate;
import static frc.robot.Constants.DrivebaseConstants.AutonMaxVelocity;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Systems;
import frc.robot.Subsystems.Field;
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
    25, // TODO: Convert these to double - AutonMaxAngularRate * 0.4,
    5.0); //AutonMaxAngularRate);

  private final ProfiledPIDController dController;
  private final ProfiledPIDController thetaController;

  private Drivebase drivebase = Systems.getDrivebase();
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final boolean turnFirst;
  private final Field field = Systems.getField();

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        // LEDSubsystem ledSubsystem,
        boolean turnFirst) {
    this(drivebase, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, turnFirst);
  }

  public DriveToPoseCommand(
        Drivebase drivebase,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        // LEDSubsystem ledSubsystem,
        boolean turnFirst) {
    this.drivebase = drivebase;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.turnFirst = turnFirst;

    dController = new ProfiledPIDController(D_kP, D_kI, D_kD, xyConstraints);
    dController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivebase);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;

    thetaController.setGoal(pose.getRotation().getRadians());
    dController.setGoal(0.0);
    }

  public boolean atGoal() {
    return dController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    dController.reset(Math.sqrt(Math.pow(goalPose.getX() - robotPose.getX(), 2) + Math.pow(goalPose.getY() - robotPose.getY(), 2)));
  }

  @Override
  public void execute() {

    var robotPose = poseProvider.get();// Drive to the goal

    double distance = Math.sqrt(Math.pow(goalPose.getX() - robotPose.getX(), 2) + Math.pow(goalPose.getY() - robotPose.getY(), 2));
    double x_diff = (goalPose.getX() - robotPose.getX()) / distance;
    double y_diff = (goalPose.getY() - robotPose.getY()) / distance;

    var distancePower = dController.calculate(distance);
    if (dController.atGoal()) {
      distancePower = 0;
      System.out.println("*****************");
      System.out.println("At Distance Goal");
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    if(turnFirst){
      if(!thetaController.atGoal()){
        distancePower = 0;
      }
    }else{
      if(!dController.atGoal()){
        omegaSpeed = 0;
      }
    }

    // Translate teh power into the unit direction for x and y
    double xSpeed = distancePower * x_diff;
    double ySpeed = distancePower * y_diff;


    drive(xSpeed, ySpeed, omegaSpeed);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive(0, 0, 0);
  }

  public void drive(double xSpeed, double ySpeed, double omegaSpeed){
    // drivebase.setControl(drivebase.getDriveFieldCentric()
    drivebase.setControl(
      drivebase.getDriveFieldCentric()
      // drivebase.getDriveRobotCentric()
      .withVelocityX(xSpeed)  // Set X velocity (forward/backward speed in m/s)
      .withVelocityY(ySpeed)  // Set Y velocity (sideways speed in m/s)
      .withRotationalRate(omegaSpeed)
      );
  }

}