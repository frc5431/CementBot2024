package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class Constants {
  public static final boolean isGals = false;
  public static double vortexStallTorque = 3.6;
  public static double neoStallTorque = 2.6;
  public static double neo550StallTorque = 0.97;

  public static class idsorsmth {

    public static final int idorsmth = 0;
    public static final boolean isTauseefCool = true;
  }
  
  public static class ControllerConstants {
    public enum ControlStates {
        CLEAN, CORAL,
    }

    public static final int driverPort = 0;
    public static final int operatorPort = 1;

    public static final boolean using8BitDo = false;

    public static final double deadzone = 0.15;
    public static final double triggerThreshold = 0.5;

}

public static class VisionConstants {

        public static final double p = 0.05;
        public static final double i = 0;
        public static final double d = 0.005;


        public static final boolean useVisionPeriodic = true;

        public static final String cameraName = "limelight";
        public static final Distance centLLForwardOffset = Units.Inches.of(0);
        public static final Distance centLLRightOffset = Units.Inches.of(0);
        public static final Distance centLLUpOffset = Units.Inches.of(0);
        public static final Angle centLLRollOffset = Units.Degrees.of(0);
        public static final Angle centLLPitchOffset = Units.Degrees.of(0);
        public static final Angle centLLYawOffset = Units.Degrees.of(180);

        public static final Distance rightLLForwardOffset = Units.Inches.of(0);
        public static final Distance rightLLRightOffset = Units.Inches.of(0);
        public static final Distance rightLLUpOffset = Units.Inches.of(0);
        public static final Angle rightLLRollOffset = Units.Degrees.of(0);
        public static final Angle rightLLPitchOffset = Units.Degrees.of(0);
        public static final Angle rightLLYawOffset = Units.Degrees.of(0);

        public static final int centerTagPipeline = 0;
        public static final int rightTagPipeline = 0;

        public static final Distance rightPipeOffset = Units.Inches.of(6);
        public static final Distance leftPipeOffset = Units.Inches.of(-6);
        public static final Distance pipeScoreOffset = Units.Inches.of(6);
        public static final Distance centerScoreOffset = Units.Inches.of(2);
        public static final Distance centerOffset = Units.Inches.of(2);
        public static final Distance allowedError = Units.Inches.of(1);

        public static final LinearVelocity alignYVelocity = Units.FeetPerSecond.of(1);
        public static final LinearVelocity alignXVelocity = Units.FeetPerSecond.of(1);
        public static final AngularVelocity alignThetaVelocity = Units.RadiansPerSecond.of(0.0);
        public static final ChassisSpeeds alignXSpeed = new ChassisSpeeds(alignXVelocity, Units.FeetPerSecond.of(0),
                alignThetaVelocity);
        public static final ChassisSpeeds alignYSpeed = new ChassisSpeeds(Units.FeetPerSecond.of(0), alignYVelocity,
                alignThetaVelocity);

        public static final double highTrustStds = 0.1;
        public static final double servicableTrustStds = 0.25;
        public static final double defaultTrustStds = 0.5;
        public static final double decreasedTrustStds = 2;

        public static final double lowTrustStds = 5;
        public static final double badTrustStds = 8;
        public static final double dismalTrustStds = 15;
        public static final double abysmalTrustStds = 16;
        public static final double noTrustStds = 9999;

        /**
         * idk what unit this is
         * spectrum did 0.025
         */
        public static final double minSizeRejection = 0.025;
        public static final AngularVelocity maxRadPerSec = Units.RadiansPerSecond.of(1.6);
        public static final AngularVelocity lowTrustRadPerSec = Units.RadiansPerSecond.of(0.5);

        public static final Distance visionRejectDistance = Units.Meters.of(1);
        /**
         * Meters per Second
         */
        public static final double velocityLowTrustThreshold = 0.2;

    }

    public static class AutonConstants {
        // The PID values from last year
        public static final PIDConstants translationPID = new PIDConstants(2, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(.1, 0, .01);
    }

  public static class DrivebaseConstants {
        public static final AngularVelocity MaxAngularRate = Units.RotationsPerSecond.of(0.5);
        public static final Distance robotLength = Units.Inches.of(28);
        public static final AngularVelocity AngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final AngularVelocity VisionAngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);

        public static final AngularVelocity AutoAngularDeadzone = DrivebaseConstants.MaxAngularRate.times(0.1);
        public static final AngularVelocity AutonMaxAngularRate = Units.RotationsPerSecond.of(0.5);
         
    }

  public static class TunerConstatns {
     // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.59).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(80.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
        );
    private static final CANcoderConfiguration 
    encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 12.8;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 13;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 5;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.07275390625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(11.5);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 10;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.469970703125);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(11.5);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    // Back Left
    private static final int kBackLeftDriveMotorId = 8;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 12;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.34716796875);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-11.5);
    private static final Distance kBackLeftYPos = Inches.of(10);

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 9;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.360107421875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-11.5);
    private static final Distance kBackRightYPos = Inches.of(-10);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**



    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
      }
    }
}


