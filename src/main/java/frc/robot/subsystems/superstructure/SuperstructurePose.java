package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.LoggedTunableNumber;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Represents a pose of the superstructure with suppliers for dynamic position control.
 * Uses suppliers to enable both dynamic and static positions.
 */
public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> endEffectorAngle, Supplier<Rotation2d> intakeAngle) {
    /**
     * Preset poses for the superstructure.
     * Provides two ways to create poses:
     * 1. Dynamic poses using suppliers for runtime-calculated positions
     * 2. Static poses using fixed values that can be tuned through NetworkTables
     */
    @Getter
    @RequiredArgsConstructor
    enum Preset {
        CORAL_STOW("Coral Stow",0.5, 45, 0),
        ALGAE_STOW("Algae Stow", 0.5, 45, 0),
        START("Start", 0.0, 0, 0),
        L1_INTAKE_SIDE("L1 Intake Side", 0.5, 0, 0),
        L1_SHOOT_SIDE("L1 Shoot Side", 0.5, 0, 0),
        L2("L2", 0.5, 0, 0),
        L3("L3",0.7,45,45),
        L4("L4", 0.7, 45, 45),
        NET_SCORE("Net Score", 0.5, 0, 0),
        P1("P1", 0.5, 0, 0),
        P2("P2", 0.5, 0, 0),
        CORAL_GROUND_INTAKE("Coral Ground Intake", 0.0, 0, 90),
        AVOID("Avoid", 0.5, 0, 0);


        
        private final SuperstructurePose pose;

        Preset(DoubleSupplier elevatorHeight, DoubleSupplier endEffectorAngle, DoubleSupplier intakeAngle) {
            this(
                new SuperstructurePose(
                    elevatorHeight, () -> Rotation2d.fromDegrees(endEffectorAngle.getAsDouble()), () -> Rotation2d.fromDegrees(intakeAngle.getAsDouble())));
          }
      
        Preset(String name, double elevatorHeight, double endEffectorAngle, double intakeAngle) {
            this(
                new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
                new LoggedTunableNumber("Superstructure/" + name + "/Pivot", endEffectorAngle),
                new LoggedTunableNumber("Superstructure/" + name + "/Intake", intakeAngle));
        }

        
    }
} 