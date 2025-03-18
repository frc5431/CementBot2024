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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AlignReefCommandTake2 extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private ProfiledPIDController pid;

    public AlignReefCommandTake2(boolean rightTrue) {
        
        // set constraints
        double maxVelocity = 1.0; // units: m/s
        double maxAcceleration = 1.0; // units m/s2
        TrapezoidProfile.Constraints xyConstants = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

        pid = new ProfiledPIDController(
            VisionConstants.p, VisionConstants.i, VisionConstants.d, xyConstants);

        pid.setGoal( rightTrue ? VisionConstants.rightPipeOffset.in(Units.Inches) : VisionConstants.leftPipeOffset.in(Units.Inches));
        pid.setTolerance(VisionConstants.allowedError.in(Units.Inches));

        if (vision.OnlyIfNullChecker()) {

            addCommands(
                    
                    new ParallelRaceGroup(
                        // vision.solidLimelight(),
                    //    drivebase.driveRobotCentric(rightTrue ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1)),
                       drivebase.driveRobotCentricCommand(new ChassisSpeeds(
                            pid.calculate(vision.getCameraXDistance().in(Units.Inches),
                                 rightTrue ? VisionConstants.rightPipeOffset.in(Units.Inches) 
                                 : VisionConstants.leftPipeOffset.in(Units.Inches)), 0, 0)), 
                      
                      new WaitUntilCommand(() -> vision.getPipeAlignDist(rightTrue))// || vision.getPipeAlignOverShoot(rightTrue))
                    ),
                    vision.blinkLimelights().withTimeout(0.5)

                    );


                   

        } else {
            addCommands(new PrintCommand("No Vision 2"));
        }
      

        addRequirements(drivebase, vision);
    } 
}