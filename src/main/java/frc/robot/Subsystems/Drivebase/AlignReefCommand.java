package frc.robot.Subsystems.Drivebase;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.VisionConstants;

public class AlignReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private PIDController pid = new PIDController(VisionConstants.p, VisionConstants.i, VisionConstants.d);

    public AlignReefCommand(boolean rightTrue) {
        

        if (vision.OnlyIfNullChecker()) {

            addCommands(
                    
                    new ParallelRaceGroup(
                        // vision.solidLimelight(),
                    //    drivebase.driveRobotCentric(rightTrue ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1)),
                       drivebase.driveRobotCentric(new ChassisSpeeds(
                            pid.calculate(vision.getCameraXDistance().in(Units.Inches),
                                 rightTrue ? VisionConstants.rightPipeOffset.in(Units.Inches) 
                                 : VisionConstants.leftPipeOffset.in(Units.Inches)), 0, 0)).repeatedly(), 
                      
                      new WaitUntilCommand(() -> vision.getPipeAlignDist(rightTrue))// || vision.getPipeAlignOverShoot(rightTrue))
                    ),
                    vision.blinkLimelights().withTimeout(0.5)

                    );


                   

        } else {
            addCommands();
        }
      

        addRequirements(drivebase, vision);
    } // TODO: fix this

    // public AlignReefCommand() {
    // if (vision.OnlyIfNullChecker()) {
    // addCommands(
    // drivebase.faceTargetCommand(vision.getBestLimelight().getMegaPose2d().getRotation()),
    // drivebase.driveRobotCenteric(
    // vision.leftOfTag() ? VisionConstants.alignXSpeed :
    // VisionConstants.alignXSpeed.times(-1))
    // .until(() -> vision.isCentered()));

    // // drivebase.driveRobotCenteric(VisionConstants.alignYSpeed).until(() ->
    // // vision.getCenterScoreDistance()));
    // } else {
    // addCommands();
    // }

    // // alongWith(candle.changeAnimationCommand(AnimationTypes.BLINK_BLUE));

    // //
    // andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

    // addRequirements(drivebase, vision);// , candle);
    // } //TODO: fix this
}
