package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;

public class ZeroElevatorCommand extends SequentialCommandGroup {
    public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem) {
        addRequirements(elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem);
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.NEUTRAL)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorState(ElevatorSubsystem.WantedState.ZERO))
                )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
