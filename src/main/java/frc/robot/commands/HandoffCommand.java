package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AmperConstants.AmperModes;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class HandoffCommand extends SequentialCommandGroup {

    public HandoffCommand(Intake intake, Pivot intakePivot, Pivot amperPivot, Amper amper) {
        addCommands(
                new RunAnglerCommand(() -> Constants.IntakeConstants.anglerConstants.mainAngle.in(Units.Degrees), intakePivot, TerminationCondition.SETPOINT_REACHED).alongWith(
                        new RunAnglerCommand(() -> Constants.AmperConstants.anglerConstants.minAngle.in(Units.Degrees), amperPivot, TerminationCondition.SETPOINT_REACHED)).withTimeout(.5),
                Commands.race(
                        intake.runMode(IntakeModes.OUTAKE).alongWith(amper.runMode(AmperModes.INTAKE))
                        .until(() -> amper.getBeamBreakStatus().get()).withTimeout(0.5)));
    }
}