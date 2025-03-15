package frc.robot.Subsystems.Drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.Systems;
import frc.robot.Subsystems.Field;
import frc.robot.Subsystems.Limelight.Vision;

public class RotateReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private Field field = Systems.getField();
    private Rotation2d apriltagRotation;

    public RotateReefCommand(){
        addCommands(
        new ParallelRaceGroup(
        drivebase.faceAprilTag()
        //facetargetcommand
        ));
    }

}
