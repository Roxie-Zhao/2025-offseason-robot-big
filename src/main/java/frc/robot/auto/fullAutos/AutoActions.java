package frc.robot.auto.fullAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.auto.basics.FollowPath;
import frc.robot.auto.commands.AutoGroundIntakeCommand;
import frc.robot.auto.commands.ReefAimAlgaeAutoCommand;
import frc.robot.auto.commands.ReefAimCoralAutoCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.aimSequences.AutoPreShootCommand;
import frc.robot.commands.manualSequence.PreShootCommand;
import frc.robot.commands.manualSequence.PutAlgaeNetCommand;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class AutoActions {
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Swerve swerve;
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();

    public AutoActions(IndicatorSubsystem indicatorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = Swerve.getInstance();
    }

    // invoke event marker
    public void invokeCommand(String name, BooleanSupplier stopSupplier) {
        switch (name) {
            case "DEPLOY-INTAKE":
                deployIntake().until(stopSupplier).schedule();
                break;
            case "PRESHOOT":
                preShoot().until(stopSupplier).schedule();
                break;
            case "ZEROELEVATOR":
                zeroElevator().until(stopSupplier).schedule();
                break;
            case "DEPLOY-INTAKE-INIT":
                zeroAndIntake().until(stopSupplier).schedule();
                //deployIntake().until(stopSupplier).schedule();
                break;
        }
    }

    public Command followPath(PathPlannerPath path, boolean angleLock, boolean requiredOnTarget, boolean resetOdometry) {
        return new FollowPath(this, swerve, path, angleLock, requiredOnTarget, resetOdometry);
    }

    public Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    public Command setL4() {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4));
    }

    public Command zeroElevator() {
        return new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem);
    }

    public Command deployIntake() {
        return new AutoGroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem);
    }

    public Command preShoot() {
        return new PreShootCommand(indicatorSubsystem, endEffectorArmSubsystem, intakeSubsystem, elevatorSubsystem);
    }

    public Command setLevel(DestinationSupplier.elevatorSetpoint setpoint) {
        return Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(setpoint));
    }

    public Command AutoAimShoot(DestinationSupplier.elevatorSetpoint setpoint, char tagChar) {
        return Commands.sequence(
                Commands.parallel(
                        setLevel(setpoint),
                        new ReefAimCoralAutoCommand(elevatorSubsystem, tagChar),
                        new AutoPreShootCommand(indicatorSubsystem, endEffectorArmSubsystem, intakeSubsystem, elevatorSubsystem)
                ),
                new WaitCommand(0.2),
                new ShootCommand(indicatorSubsystem, endEffectorArmSubsystem),
                new WaitCommand(0.05),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(
                        RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get())));
    }

    public Command homeEverything() {
        return Commands.parallel(Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME)),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorState(ElevatorSubsystem.WantedState.IDLE)));
    }

    public boolean intakerHasCoral() {
        return intakeSubsystem.hasCoral();
    }

    public EndEffectorArmSubsystem.SystemState getEESystemState() {
        return endEffectorArmSubsystem.getSystemState();
    }

    public boolean isIntakeFinished() {
        return endEffectorArmSubsystem.hasCoral();
    }

    public Command zeroAndIntake() {
        return Commands.sequence(
                new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem),
                new WaitUntilCommand(() -> (elevatorSubsystem.getSystemState() != ElevatorSubsystem.SystemState.ZEROING)),
                new AutoGroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem));
    }

    public Command intakeAlgae() {
        return Commands.parallel(
                Commands.runOnce(() ->
                        DestinationSupplier.getInstance().setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING)),
                new AutoPreShootCommand(indicatorSubsystem, endEffectorArmSubsystem, intakeSubsystem, elevatorSubsystem),
                new ReefAimAlgaeAutoCommand(elevatorSubsystem, indicatorSubsystem)
        ).finallyDo(() -> {
            endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
            elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
        });
    }

    public Command shootAlgaeNet() {
        return new PutAlgaeNetCommand(1, endEffectorArmSubsystem, elevatorSubsystem, intakeSubsystem, indicatorSubsystem);
    }
}
