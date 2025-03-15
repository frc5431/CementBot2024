package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstatns.TunerSwerveDrivetrain;
import frc.robot.Subsystems.Drivebase.Drivebase;  // Assume you have a DriveSubsystem to control your robot// The class you provided

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class RotateToAngleCommand extends Command {

    private final Drivebase drivebase;
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle;
    private final double targetAngle;
    private SwerveControlParameters swerveControlParameters; 

    // Constructor, take in DriveSubsystem, target angle, and Xbox controller
    public RotateToAngleCommand(Drivebase drivebase, double targetAngle) {
        this.drivebase = drivebase;
        this.targetAngle = targetAngle;
        
        // Creates an instance of FieldCentricFacingAngle
        this.fieldCentricFacingAngle = new FieldCentricFacingAngle();
        addRequirements(drivebase);  // Specify that this command uses the drive subsystem
    }

    // Initialize is called once when the command is scheduled
@Override
    public void initialize() {
        fieldCentricFacingAngle
            .withTargetDirection(new Rotation2d(Math.toRadians(targetAngle)))  // Set the desired target angle
            .withVelocityX(0)  // Set forward velocity to 0
            .withVelocityY(0)  // Set sideways velocity to 0
            .withHeadingPID(1.0, 0.0, 0.0);  // Example PID gains for heading control
    }

    // Execute is called repeatedly when the command is running
    @Override
    public void execute() {
        // Apply the command to the drive system
        // drivebase.applyFieldCentricRequest(fieldCentricRequest);
        drivebase.updateControlParameters();
        fieldCentricFacingAngle.apply(drivebase.getControlParameters(), drivebase.getModules());
    }

    // IsFinished is called to check if the command has finished
    @Override
    public boolean isFinished() {
        return Math.abs(drivebase.getRotation3d().toRotation2d().getDegrees() - targetAngle) < 5.0;  // Example: finished when within 5 degrees
    }

    // End is called once the command ends
    @Override
    public void end(boolean interrupted) {
        // You can stop the robot or reset anything here if needed
    }
}