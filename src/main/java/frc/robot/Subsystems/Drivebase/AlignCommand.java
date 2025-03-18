package frc.robot.Subsystems.Drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Systems;
import frc.robot.Subsystems.Field;
import frc.robot.Subsystems.Limelight.Vision;

public class AlignCommand extends Command {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private boolean rightTrue;
    private boolean bad = false;

    public AlignCommand(boolean rightTrue) {
        addRequirements(drivebase, vision);
        this.rightTrue = rightTrue;
    }

    @Override
    public void initialize() {
        bad = false;
        // vision.isAlginAble = vision.OnlyIfNullChecker();
        if (!(Field.isRedTag(vision.getBestLimelight().getClosestTagID()) == Field.isRed())
                    && Field.isReef(vision.getBestLimelight().getClosestTagID())) {
            System.out.println("Ended in Intalizatiobion");
            this.end(true);
        }

    }

    @Override
    public void execute() {
        System.out.println("Execute");
        if ((Field.isRedTag(vision.getBestLimelight().getClosestTagID()) == Field.isRed())
        && Field.isReef(vision.getBestLimelight().getClosestTagID())) {
            System.out.println("Alginable");
            drivebase.driveRobotCentric(
                    new ChassisSpeeds(rightTrue ? 0.4 : -0.4, 0, 0));
        } else {
            System.out.println(":(");
            bad = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.driveRobotCentric(
                new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return vision.getPipeAlignDist(rightTrue) || vision.getPipeAlignOverShoot(rightTrue) || bad;
    }
}
