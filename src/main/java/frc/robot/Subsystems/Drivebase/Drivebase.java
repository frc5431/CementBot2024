package frc.robot.Subsystems.Drivebase;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TunerConstatns;
import frc.robot.Systems;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DrivebaseConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Field;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.commands.RotateToAngleCommand;
import frc.team5431.titan.swerve.SwerveConstants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Drivebase extends frc.robot.Constants.TunerConstatns.TunerSwerveDrivetrain implements Subsystem {
    private static SwerveControlParameters controlParameters; // TODO: ADD IN VALUES

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    // TODO: ADD IN NULL VALUES
    // TODO: GIVE DRIVER DPAD ALIGNMENT

    private SwerveRequest.SwerveDriveBrake xLock = new SwerveDriveBrake();
    private SwerveRequest.FieldCentricFacingAngle driverFieldCentricFacingAngle = new FieldCentricFacingAngle()
            .withMaxAbsRotationalRate(DrivebaseConstants.AutonMaxAngularRate)
            .withHeadingPID(5, 0, 0.1)
            .withRotationalDeadband(DrivebaseConstants.AngularDeadzone)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    public SwerveRequest.FieldCentricFacingAngle getDriverFieldCentricFacingAngle() {
        return driverFieldCentricFacingAngle;
    }
    public SwerveRequest.RobotCentric driveRobotCentric = new RobotCentric()
            .withDeadband(0)
            .withRotationalDeadband(0) // Add a 10%
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    public SwerveRequest.RobotCentric getDriveRobotCentric() {
        return driveRobotCentric;
    }

    public SwerveRequest.FieldCentric driveFieldCentric = new FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0) // Add a 10%
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public SwerveRequest.FieldCentric getDriveFieldCentric() {
        return driveFieldCentric;
    }
    private SwerveRequest.RobotCentricFacingAngle driveRobotCentricFacingAngle = new RobotCentricFacingAngle()
            .withMaxAbsRotationalRate(DrivebaseConstants.AutonMaxAngularRate)
            .withRotationalDeadband(DrivebaseConstants.AutoAngularDeadzone)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    public SwerveRequest.RobotCentricFacingAngle getDriveRobotCentricFacingAngle() {
        return driveRobotCentricFacingAngle;
    }
    private SwerveRequest.ForwardPerspectiveValue perspectiveValue = ForwardPerspectiveValue.OperatorPerspective;

    public SwerveRequest.ForwardPerspectiveValue getPerspectiveValue() {
        return perspectiveValue;
    }

    public SwerveRequest.RobotCentric getVisionRobotCentric() {
        return visionRobotCentric;
    }

    private SwerveRequest.FieldCentric driverControl = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstatns.kSpeedAt12Volts.times(0.05))
            .withRotationalDeadband(DrivebaseConstants.MaxAngularRate.times(0.1).in(RadiansPerSecond)) // Add a 10%
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public SwerveRequest.RobotCentric visionRobotCentric = new RobotCentric()
            .withRotationalDeadband(DrivebaseConstants.VisionAngularDeadzone);

    public SwerveRequest.FieldCentric getDriverControl() {
        return driverControl;
    }

    SwerveModuleState[] states = this.getState().ModuleStates;
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Robot Pose", Pose2d.struct).publish();

    StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Chassis Speed", ChassisSpeeds.struct).publish();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *                            Drivetrain-wide constants for the swerve drive
     * @param modules
     *                            Constants for each specific module
     */
    public Drivebase(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        configureAutoBuilder();
        controlParameters = new SwerveControlParameters();
        controlParameters.kinematics = this.getKinematics();
        controlParameters.kMaxSpeedMps = 4.0;
        controlParameters.currentPose = new Pose2d(0, 0, new Rotation2d());
        controlParameters.currentChassisSpeed = new ChassisSpeeds(0, 0, 0);
        controlParameters.timestamp = Timer.getFPGATimestamp();
        controlParameters.updatePeriod = 0.02;
    }

    public void updateControlParameters() {
        controlParameters.currentPose = this.getRobotPose();
        controlParameters.currentChassisSpeed = this.getChassisSpeeds();
        controlParameters.timestamp = Timer.getFPGATimestamp();
    }

    public static SwerveControlParameters getControlParameters() {
        return controlParameters;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants
     *                                  Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency
     *                                  The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation
     *                                  The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation
     *                                  The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules
     *                                  Constants for each specific module
     */
    private Field field = new Field();
    private Vision vision = Systems.getVision();

    public Drivebase(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        configureAutoBuilder();
    }

    public void resetGyro() {
        this.resetRotation(Rotation2d.kZero);
    }

    public Command zeroGyro() {
        return new InstantCommand(() -> resetGyro(), this);
    }

    public void zeroEverything(){
        this.resetRotation(aprilTagRotation);
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                    this::getRobotPose,
                    this::resetPose,
                    this::getChassisSpeeds,
                    (speeds) -> driveRobotCentric(speeds),
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            AutonConstants.translationPID,
                            AutonConstants.rotationPID),
                    RobotConfig.fromGUISettings(),
                    () -> Field.isRed(),
                    this);

        } catch (Exception e) {
            DriverStation.reportError("Auton Config Issue", e.getStackTrace());
        }
    }

    /**
     * The function `getRobotPose` returns the robot's pose after checking and
     * updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after
     *         calling the
     *         `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        return keepPoseOnField(pose);
    }

    // Keep the robot on the field
    private Pose2d keepPoseOnField(Pose2d pose) {

        double halfBot = DrivebaseConstants.robotLength.div(2).in(Meters);
        double x = pose.getX();
        double y = pose.getY();

        // WARNING: IF ANTHING BAD IS EVER HAPPENING, IM NOT SURE THIS IS RIGHT
        double newX = MathUtil.clamp(x, halfBot, (Field.getFieldlength().in(Meters) - halfBot));
        double newY = MathUtil.clamp(y, halfBot, (Field.getFieldlength().in(Meters) - halfBot));

        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }
        return pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request
     *                Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command alignWheelsCommand(Rotation2d direction) {
        return run(() -> new SwerveRequest.PointWheelsAt().withModuleDirection(direction));
    }

    /**
     * @param chassisSpeeds
     * @return
     *         If we have issues this is a good place to start. Not confident on the
     *         end command
     */
    public Command driveRobotCentric(ChassisSpeeds chassisSpeeds) {
        return run(() -> this.setControl(visionRobotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond)));
        // return run(() ->
        // setControl(visionRobotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond).withVelocityY(chassisSpeeds.vyMetersPerSecond)));
    }

    public Command stopRobotCentric() {
        return new InstantCommand(() -> this
                .setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
    }

    public Command faceTargetCommand(Rotation2d faceDirection) {
        return applyRequest(() -> driverFieldCentricFacingAngle.withTargetDirection(faceDirection));
    }

    public Command randomTst() {
        return applyRequest(() -> driverFieldCentricFacingAngle.withTargetDirection(new Rotation2d(Units.Degrees.of(60))));
    }
    // public Command faceAprilTag(){
    // return new RotateToAngleCommand(this, this.getAprilTagRotation());
    // }

    public Command faceAprilTag() {
        Rotation2d rotationSetpoint = this.getAprilTagRotation();
        SmartDashboard.putNumber("April Tag Setpoint", rotationSetpoint.getDegrees());

        return new SequentialCommandGroup(

                faceTargetCommand(rotationSetpoint)
                        .until(() -> getRotation3d().toRotation2d().minus(rotationSetpoint).getDegrees() < 5.0)

        );
    }

    public Command getAprilTagRotatiCommand() {
        return new InstantCommand(() -> this.getAprilTagRotation());
    }

    // public FieldCentricFacingAngle testcentricangle(){

    // }

    public Rotation2d aprilTagRotation;

    public Rotation2d getAprilTagRotation() {
        Pose3d tagpose = field.getAprilTagPose3d(vision.getBestLimelight().getClosestTagID());
        if (tagpose != null) {
            aprilTagRotation = tagpose.getRotation().toRotation2d();
            return tagpose.getRotation().toRotation2d();
        }
        return new Rotation2d();
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        SmartDashboard.putNumber("Gyro", this.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Drivebase Rotation", this.getRotation3d().getMeasureZ().in(Degrees));
        SmartDashboard.putNumber("April Tag Rot", this.getAprilTagRotation().getDegrees());
        // SmartDashboard.putData("Swerve Pose", (Sendable) this.getRobotPose());

        publisher.set(states);
        posePublisher.set(getRobotPose());
        speedsPublisher.set(getChassisSpeeds());
        updateSimState(0.02, 12.0); // Added so I can use swerve in simulation

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

}