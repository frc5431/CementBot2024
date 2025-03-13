package frc.robot.Subsystems.Drivebase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.Constants.VisionConstants;

public class AlignReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();

    public AlignReefCommand(boolean rightTrue) {

        if (vision.OnlyIfNullChecker()) {
            addCommands(
                    vision.solidLimelight(),
                    drivebase.faceTargetCommand(vision.getBestLimelight().getMegaPose2d().getRotation()),
                    drivebase.driveRobotCentric(rightTrue ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1)),
                    new WaitUntilCommand(() -> vision.getPipeAlignDist(rightTrue)),
                    drivebase.driveRobotCentric(VisionConstants.alignYSpeed),
                    new WaitUntilCommand(() -> vision.getPipeAlignDist(rightTrue)),
                    drivebase.stopRobotCentric(),
                    vision.blinkLimelights().withTimeout(1)
                    );
        } else {
            addCommands();
        }

        addRequirements(drivebase, vision);
    } //TODO: fix this

    // public AlignReefCommand() {
    //     if (vision.OnlyIfNullChecker()) {
    //         addCommands(
    //                 drivebase.faceTargetCommand(vision.getBestLimelight().getMegaPose2d().getRotation()),
    //                 drivebase.driveRobotCenteric(
    //                         vision.leftOfTag() ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1))
    //                         .until(() -> vision.isCentered()));

    //         // drivebase.driveRobotCenteric(VisionConstants.alignYSpeed).until(() ->
    //         // vision.getCenterScoreDistance()));
    //     } else {
    //         addCommands();
    //     }

    //     // alongWith(candle.changeAnimationCommand(AnimationTypes.BLINK_BLUE));

    //     // andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

    //     addRequirements(drivebase, vision);// , candle);
    // } //TODO: fix this
}
