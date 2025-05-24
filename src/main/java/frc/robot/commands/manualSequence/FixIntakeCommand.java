package frc.robot.commands.manualSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;

public class FixIntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;

    public FixIntakeCommand(ElevatorSubsystem elevatorSubsystem,
                            IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(elevatorSubsystem, endEffectorArmSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        if (elevatorSubsystem.elevatorReady(0.01)) {
            if (elevatorSubsystem.getWantedPosition() == 0.25) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
                elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
                endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
            } else if (elevatorSubsystem.getWantedPosition() == HOME_EXTENSION_METERS.get()) {
                elevatorSubsystem.setElevatorPosition(0.25);//TODO: Constants
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
                endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_INTAKE);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
    }

    @Override
    public boolean isFinished() {
        return endEffectorArmSubsystem.hasCoral();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}