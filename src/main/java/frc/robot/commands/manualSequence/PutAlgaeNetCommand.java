package frc.robot.commands.manualSequence;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.drivers.GamepieceTracker;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;

public class PutAlgaeNetCommand extends ParallelCommandGroup {
    public PutAlgaeNetCommand(CommandXboxController driverController, EndEffectorArmSubsystem endeffectorArmSubsystem,
                              ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, IndicatorSubsystem indicatorSubsystem) {
        addRequirements(endeffectorArmSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_NET_PRESHOOT)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.ALGAE_NET_EXTENSION_METER.get())),
                        Commands.sequence(
                                Commands.waitUntil(() -> driverController.rightTrigger().getAsBoolean()),
                                Commands.runOnce(() -> endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_NET_SHOOT)),
                                Commands.waitSeconds(100)
                        )
                ).finallyDo(() -> {
                    if ((!GamepieceTracker.getInstance().isEndeffectorHasCoral() && !GamepieceTracker.getInstance().isEndeffectorHasAlgae())) {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get());
                    } else {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
                    }
                    endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
                })
        );
    }

    public PutAlgaeNetCommand(double time, EndEffectorArmSubsystem endeffectorArmSubsystem,
                              ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, IndicatorSubsystem indicatorSubsystem) {
        addRequirements(endeffectorArmSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_NET_PRESHOOT)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.ALGAE_NET_EXTENSION_METER.get())),
                        Commands.sequence(
                                Commands.waitSeconds(time),
                                Commands.runOnce(() -> endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_NET_SHOOT)),
                                Commands.waitSeconds(1)
                        )
                ).finallyDo(() -> {
                    if ((!GamepieceTracker.getInstance().isEndeffectorHasCoral() && !GamepieceTracker.getInstance().isEndeffectorHasAlgae())) {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get());
                    } else {
                        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
                    }
                    endeffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
                })
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
