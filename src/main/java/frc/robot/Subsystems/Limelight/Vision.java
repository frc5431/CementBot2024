package frc.robot.Subsystems.Limelight;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivebase.Drivebase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Limelight.LimelightHelpers;
import frc.robot.Subsystems.Limelight.LimelightHelpers.RawFiducial;
import frc.robot.Subsystems.Limelight.VisionUtil.LimelightLogger;
import frc.robot.Subsystems.Limelight.VisionUtil.VisionConfig;
import frc.robot.Subsystems.Field;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.team5431.titan.core.misc.Calc;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.text.DecimalFormat;
import java.util.ArrayList;
// import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.Systems;

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision}
     * {@code allLimelights} &
     * {@code poseLimelights}
     */

    StructArrayPublisher<AprilTag> aprilPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AprilTags", new AprilTagStruct()).publish();

    StructPublisher<Pose3d> rawPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Raw Pose", Pose3d.struct).publish();

    StructPublisher<Pose2d> megaPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Mega Pose", Pose2d.struct).publish();


    private Trigger reefAlignment = new Trigger(
            Systems.getDriver().rightBumper().or(Systems.getDriver().leftBumper()));

    /* Pose Estimation Constants */ // 2.3;

    public Trigger getReefAlignment() {
        return reefAlignment;
    }

    public void setReefAlignment(Trigger reefAlignment) {
        this.reefAlignment = reefAlignment;
    }

    // Increase these numbers to trust global measurements from vision less.
    public static double VISION_STD_DEV_X = 0.5;
    public static double VISION_STD_DEV_Y = 0.5;
    public static double VISION_STD_DEV_THETA = 99999999;
    public boolean isAlginAble = false;


    public static final Matrix<N3, N1> visionStdMatrix = VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y,
            VISION_STD_DEV_THETA);

    // TODO: deal with this
    private static Drivebase drivebase = Systems.getDrivebase();

    // TODO: we only have one for now
    /* Limelights */
    public final LimelightHelpers.VisionHelper centLL3 = new LimelightHelpers().new VisionHelper(
            VisionConfig.LEFT_LL, VisionConstants.centerTagPipeline, VisionConfig.centerConfig);

    // public final LimelightLogger leftLogger = new LimelightLogger("Left",
    // centLL3);
    // public final VisionHelper rightLL = new LimelightHelpers().new VisionHelper(
    // VisionConfig.RIGHT_LL,
    // VisionConstants.rightTagPipeline,
    // VisionConfig.RIGHT_CONFIG);
    // public final LimelightLogger rightLogger = new LimelightLogger("Right",
    // rightLL);
    public final LimelightHelpers.VisionHelper[] allLimelights = { centLL3 };
    public final LimelightHelpers.VisionHelper[] poseLimelights = {
            centLL3
    };

    private final DecimalFormat df = new DecimalFormat();

    // @AutoLogOutput(key = "Vision/is_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<LimelightHelpers.Trio<Pose3d, Pose2d, Double>> autonPoses = new ArrayList<LimelightHelpers.Trio<Pose3d, Pose2d, Double>>();

    private boolean isAligning = false;

    public void setAligning(boolean isAligning) {
        this.isAligning = isAligning;
    }

    public Vision() {
        setName("Vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (LimelightHelpers.VisionHelper visionHelper : allLimelights) {
            visionHelper.setLEDMode(false);
        }

        aprilPublisher.accept(field.getAprilTagFieldLayout().getTags().toArray(AprilTag[]::new));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Reef Tag Scan", this.OnlyIfNullChecker());
        SmartDashboard.putBoolean("Is Alginable", isAlginAble);

        SmartDashboard.putBoolean("Right Reef Align", this.getPipeAlignDist(true)); // Yaw should be 0 when intake faces
                                                                                    // red
        SmartDashboard.putBoolean("Left Reef Align", this.getPipeAlignDist(false));
        SmartDashboard.putBoolean("Reef Score Dist", this.getPipeScoreDist());
        SmartDashboard.putNumber("Tag Fid ID", this.getBestLimelight().getClosestTagID());

        SmartDashboard.putNumber("Tag Rotation",
                getBestLimelight().getMegaPose2d() != null
                        ? getBestLimelight().getMegaPose2d().getRotation().getDegrees()
                        : 458734057);
        SmartDashboard.putNumber("X Camera Distance", getCameraXDistance().in(Inches));
        SmartDashboard.putNumber("Y Camera Distance", getCameraYDistance().in(Inches));
        

        // ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp()
        // want to check these data
        SmartDashboard.putBoolean("Target In View", this.getBestLimelight().targetInView());
        rawPosePublisher.set(this.getBestLimelight().getRawPose3d());
        megaPosePublisher.set(this.getBestLimelight().getMegaPose2d());

        // Yaw should be 0 when intake faces red
        // alliance and manip faces blue
        // Yaw should be 180 when intake faces blue alliance and manip faces red
        double yaw = Systems.getDrivebase().getOperatorForwardDirection().getMeasure().plus(Degrees.of(180))
                .in(Degrees);
        for (LimelightHelpers.VisionHelper visionHelper : poseLimelights) {
            visionHelper.setRobotOrientation(yaw);

            if (DriverStation.isAutonomousEnabled() && visionHelper.targetInView()) {
                Pose3d botpose3D = visionHelper.getRawPose3d();
                Pose2d megaPose2d = visionHelper.getMegaPose2d();
                double timeStamp = visionHelper.getRawPoseTimestamp();
                Pose2d integratablePose = new Pose2d(megaPose2d.getTranslation(), botpose3D.toPose2d().getRotation());
                autonPoses.add(LimelightHelpers.Trio.of(botpose3D, integratablePose, timeStamp));
            }
        }

        try {
            isIntegrating = false;
            // Will NOT run in auto;
            if (DriverStation.isTeleopEnabled() && VisionConstants.useVisionPeriodic) {

                // choose LL with best view of tags and integrate from only that camera
                LimelightHelpers.VisionHelper bestLimelight = getBestLimelight();
                if (Field.isReef((bestLimelight.getClosestTagID()))) {
                    addFilteredVisionInput(bestLimelight);
                } else {
                    // TODO ADD BACK IF DOING ODOM
                    // System.out.print("SAD!: Apriltag is not matched Reef ID");
                }
            }

        } catch (Exception e) {
            // TODO ADD BACK IF DOING ODOM

            // System.out.println("Vision pose not present but tried to access it
            // [vision.periodic()]");
        }
    }

    private void addFilteredVisionInput(LimelightHelpers.VisionHelper ll) {
        double xyStds = 0.1;
        double degStds = 0.1;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getRawPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = ll.getMegaPose2d();
            LimelightHelpers.RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeeds = drivebase.getChassisSpeeds();

            // distance from current pose to vision estimated pose
            double poseError = Systems.getDrivebase().getRobotPose().getTranslation()
                    .getDistance(botpose.getTranslation());

            /* rejections */
            // reject pose if individual tag ambiguity is too high
            ll.tagStatus = "";
            for (LimelightHelpers.RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2) {
                    highestAmbiguity = tag.ambiguity;
                } else if (tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // log ambiguities
                ll.tagStatus += "Tag " + tag.id + ": " + tag.ambiguity;
                // ambiguity rejection check
                if (tag.ambiguity > 0.9) {
                    ll.sendInvalidStatus("vision: ambiguity rejection {vision:179ish}");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                ll.sendInvalidStatus("vision: bound rejection {vision:185ish}");
                return;
            } else if (Math.abs(robotSpeeds.omegaRadiansPerSecond) >= VisionConstants.maxRadPerSec
                    .in(RadiansPerSecond)) {
                // reject if we are rotating more than an amount of radians per second
                ll.sendInvalidStatus("vision: robot rotation too fast");
                return;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                ll.sendInvalidStatus("vision: height rejection");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("vision: roll/pitch rejection");
                return;
            } else if (targetSize <= VisionConstants.minSizeRejection) {
                ll.sendInvalidStatus("vision: size rejection");
                return;
            }
            /* CONFIDENCE (integration/standard devation) Evaluation */
            // if almost stationary and extremely close to tag
            else if (robotSpeeds.vxMetersPerSecond
                    + robotSpeeds.vyMetersPerSecond <= VisionConstants.velocityLowTrustThreshold
                    && targetSize > 0.4) {
                ll.sendValidStatus("Vision: Stationary close integration");
                xyStds = VisionConstants.highTrustStds;
                degStds = VisionConstants.highTrustStds;
            } else if (multiTags && targetSize > 0.05) {
                ll.sendValidStatus("Vision: Multi integration");
                xyStds = VisionConstants.servicableTrustStds;
                degStds = VisionConstants.badTrustStds;
                if (targetSize > 0.09) {
                    ll.sendValidStatus("Vision: Strong Multi integration");
                    xyStds = VisionConstants.highTrustStds;
                    degStds = VisionConstants.highTrustStds;
                }
            } else if (targetSize > 0.8 && poseError < 0.5) {
                ll.sendValidStatus("Close integration");
                xyStds = VisionConstants.defaultTrustStds;
                degStds = VisionConstants.abysmalTrustStds;
            } else if (targetSize > 0.1 && poseError < 0.3) {
                ll.sendValidStatus("Proximity integration");
                xyStds = VisionConstants.decreasedTrustStds;
                degStds = VisionConstants.noTrustStds;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = VisionConstants.noTrustStds;
                degStds = VisionConstants.noTrustStds;
            } else {
                System.out.println("Rejected, get better");
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = VisionConstants.dismalTrustStds;
            }

            if (robotSpeeds.omegaRadiansPerSecond >= VisionConstants.lowTrustRadPerSec.in(RadiansPerSecond)) {
                degStds = VisionConstants.dismalTrustStds;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Systems.getDrivebase().setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());
            Systems.getDrivebase().addVisionMeasurement(integratedPose, timeStamp);
        } else {
            ll.tagStatus = "no tags";
            ll.sendInvalidStatus("Vision: no tag found rejection");
        }
    }

    public void autonResetPoseToVision() {
        boolean reject = true;
        boolean firstSuccess = false;
        double batchSize = 5;
        for (int i = autonPoses.size() - 1; i > autonPoses.size() - (batchSize + 1); i--) {
            LimelightHelpers.Trio<Pose3d, Pose2d, Double> poseInfo = autonPoses.get(i);
            boolean success = resetPoseToVision(
                    true, poseInfo.getFirst(), poseInfo.getSecond(), poseInfo.getThird());
            if (success) {
                if (i == autonPoses.size() - 1) {
                    firstSuccess = true;
                }
                reject = false;
                // TODO ADD BACK IF DOING ODOM

                // System.out.println(
                // "AutonResetPoseToVision succeeded on " + (autonPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            // TODO ADD BACK IF DOING ODOM

            // System.out.println(
            // "AutonResetPoseToVision failed after "
            // + batchSize
            // + " of "
            // + autonPoses.size()
            // + " possible tries");

        }
    }

    public void resetPoseToVision() {
        LimelightHelpers.VisionHelper ll = getBestLimelight();

        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            System.out.println("target in view");

            Pose2d botpose = botpose3D.toPose2d();
            Pose2d robotPose = Systems.getDrivebase().getRobotPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                // TODO ADD BACK IF DOING ODOM

                // System.out.println(
                // "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE BAD POSE");
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) {
                // TODO ADD BACK IF DOING ODOM

                // System.out.println(
                // "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE OUT OF
                // FIELD");
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // TODO ADD BACK IF DOING ODOM

                // System.out.println(
                // "flying but idc");
                reject = false;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                // TODO ADD BACK IF DOING ODOM

                // System.out.println(
                // "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE TILTED");
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;
            // TODO ADD BACK IF DOING ODOM

            // System.out.println(
            // "ResetPoseToVision: Old Pose X: "
            // + robotPose.getX()
            // + " Y: "
            // + robotPose.getY()
            // + " Theta: "
            // + robotPose.getRotation().getDegrees());
            Systems.getDrivebase().setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Systems.getDrivebase().addVisionMeasurement(integratedPose, poseTimestamp);
            // robotpose after vision mesurments have been added
            robotPose = Systems.getDrivebase().getRobotPose();
            System.out.println(
                    "ResetPoseToVision: New Pose X: "
                            + robotPose.getX()
                            + " Y: "
                            + robotPose.getY()
                            + " Theta: "
                            + robotPose.getRotation().getDegrees());
            System.out.println("ResetPoseToVision: SUCCESS");
            return true;
        }
        System.out.println("FUCK");
        return false; // target not in view
    }

    // TODO: this doesnt work cant find best returns nothing
    public LimelightHelpers.VisionHelper getBestLimelight() {
        LimelightHelpers.VisionHelper bestLimelight = centLL3;
        // double bestScore = 0;
        // for (LimelightHelpers.VisionHelper visionHelper : poseLimelights) {
        // double score = 0;
        // // prefer LL with most tags, when equal tag count, prefer LL closer to tags
        // try {
        // visionHelper.getTagCountInView();
        // System.out.println("Exits Get Tag Count" + visionHelper.getTagCountInView());

        // score += visionHelper.getTagCountInView();
        // score += visionHelper.getTargetSize();

        // if (score > bestScore) {
        // bestScore = score;
        // bestLimelight = visionHelper;
        // }
        // } catch (NullPointerException nullPointerException) {
        // System.out.println("Get Tag Count Null");
        // break;
        // }
        // }
        return bestLimelight;
    }

    // @AutoLogOutput(key = "Vision/BestLimelight")
    public String logBestLimelight() {
        return getBestLimelight().CAMERA_NAME;
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (LimelightHelpers.VisionHelper visionHelper : poseLimelights) {
            if (visionHelper.hasAccuratePose())
                return true;
        }
        return false;
    }

    public Distance getCameraXDistance() {
        return LimelightHelpers.getCameraPose3d_TargetSpace(this.getBestLimelight().CAMERA_NAME).getMeasureX();
    }

    public Distance getCameraYDistance() {
        return LimelightHelpers.getCameraPose3d_TargetSpace(this.getBestLimelight().CAMERA_NAME).getMeasureZ();
    }

    public boolean getPipeAlignDist(boolean rightTrue) {
        return Calc.approxEquals(getCameraXDistance().in(Inches),
                rightTrue ? VisionConstants.rightPipeOffset.in(Inches) : VisionConstants.leftPipeOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean getPipeAlignOverShoot(boolean rightTrue) {
        if (rightTrue) {
            return getCameraXDistance().in(Inches) > VisionConstants.rightPipeOffset.in(Inches);
        } else {
            return getCameraXDistance().in(Inches) < VisionConstants.leftPipeOffset.in(Inches);
        }
    }

    public boolean getPipeScoreDist() {
        return Calc.approxEquals(getCameraYDistance().in(Inches),
                VisionConstants.pipeScoreOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean getPipeScoreOverShoot() { // pipeScoreOffset should be negative
        return (VisionConstants.pipeScoreOffset.in(Inches) >= getCameraYDistance().in(Inches));
    }


    public boolean leftOfTag() {
        return getCameraXDistance().in(Inches) < VisionConstants.centerOffset.in(Inches);
    }

    public boolean isCentered() {
        return Calc.approxEquals(getCameraXDistance().in(Inches),
                VisionConstants.centerOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    public boolean getCenterScoreDistance() {
        return Calc.approxEquals(getCameraYDistance().in(Inches),
                VisionConstants.centerScoreOffset.in(Inches),
                VisionConstants.allowedError.in(Inches));
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (LimelightHelpers.VisionHelper visionHelper : allLimelights) {
            visionHelper.setLimelightPipeline(pipeline);
        }
    }

    public boolean OnlyIfNullChecker() {
        try {
            SmartDashboard.putBoolean("Field is red?", Field.isRedTag(getBestLimelight().getClosestTagID()) == Field.isRed());
            SmartDashboard.putBoolean("Tag is Reef", Field.isReef(getBestLimelight().getClosestTagID()));
            return (Field.isRedTag(getBestLimelight().getClosestTagID()) == Field.isRed())
                    && Field.isReef(getBestLimelight().getClosestTagID());
        } catch (NullPointerException nullPointerException) {
            System.out.println("FAIL!");
            return false;
        }
    }

    public Field field = new Field();
    public Pose2d calculateRobotPositionFromTag(){
      // reefSelect:
      //    0 = Center
      //    1 = Right
      //    2 = Left
    double reefID = getBestLimelight().getClosestTagID();
      double distanceAway = getCameraYDistance().in(Inches);
      double distanceSide = getCameraXDistance().in(Inches);

      Pose3d pose = field.getAprilTagPose3d(reefID);
      if (pose == null) {
        System.out.println("NULL is calculate");
        return new Pose2d();};
      Translation3d tagTranslation = pose.getTranslation();
      Rotation3d tagRotation = pose.getRotation();

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

      // Step 4: Create the new Pose3d for the robot's position and rotation
      System.out.println("RAN COMMAND");
    //   return new Pose2d(newX, newY, tagRotation.toRotation2d());
    return pose.toPose2d();

    }

    public Command checkTag() {
        return new InstantCommand(()-> Field.isReef(getBestLimelight().getClosestTagID()));
    }

    /** Set all LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                () -> {
                    for (LimelightHelpers.VisionHelper visionHelper : allLimelights) {
                        visionHelper.blinkLEDs();
                    }
                },
                () -> {
                    for (LimelightHelpers.VisionHelper visionHelper : allLimelights) {
                        visionHelper.setLEDMode(false);
                    }
                })
                .withName("Vision.blinkLimelights");
    }

    /** Set left LL to blink */
    public Command solidLimelight() {
        return startEnd(
                () -> {
                    centLL3.setLEDMode(true);
                },
                () -> {
                    centLL3.setLEDMode(false);
                })
                .withName("Vision.solidLimeLight");
    }

    public Command turnOnLimelight() {
        return new InstantCommand(() -> centLL3.setLEDMode(true));
    }

    public Command commandSetPostionVision() {
        return new RunCommand(() -> resetPoseToVision(),

                this).withName("setting vision position");

    }
}