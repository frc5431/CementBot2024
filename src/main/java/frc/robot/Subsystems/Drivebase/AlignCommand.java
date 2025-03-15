// package frc.robot.Subsystems.Drivebase;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.Systems;
// import frc.robot.Subsystems.Limelight.Vision;

// public class AlignCommand extends Command {
//     private Drivebase drivebase = Systems.getDrivebase();
//     private Vision vision = Systems.getVision();
//     private PIDController pid = new PIDController(VisionConstants.p, VisionConstants.i, VisionConstants.d);
//     private boolean rightTrue;

//     public AlignCommand(boolean rightTrue) {
//         addRequirements(drivebase, vision);
//         this.rightTrue = rightTrue;
//     }

//     @Override 
//     public void initialize() {
//        //once when called 
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//        System.out.println("Aligning aligning aligning");
//     }

//     @Override
//     public void execute() {
//         drivebase.driveRobotCentric(new ChassisSpeeds(
//             pid.calculate(vision.getCameraXDistance().in(Units.Inches),
//             rightTrue ? VisionConstants.rightPipeOffset.in(Units.Inches) 
//             : VisionConstants.leftPipeOffset.in(Units.Inches)), 0, 0)).repeatedly();
//     }

//     @Override 
//     public void end(boolean interrupted) {
//         drivebase.driveRobotCentric(new ChassisSpeeds(0, 0, 0));
//     }

//     @Override
//     public boolean isFinished() {
//         return vision.getPipeAlignDist(rightTrue);
//     }
// }
