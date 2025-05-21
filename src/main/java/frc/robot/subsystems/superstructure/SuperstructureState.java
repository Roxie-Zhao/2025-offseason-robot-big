package frc.robot.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.subsystems.superstructure.SuperstructurePose.Preset;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    START(SuperstructureStateData.builder().pose(Preset.START.getPose()).build()),
    STOW(SuperstructureStateData.builder().pose(Preset.STOW.getPose()).build()),
    L3(SuperstructureStateData.builder().pose(Preset.L3.getPose()).build()),
    L3_EJECT(
        L3.getValue().toBuilder().endEffectorVolts(() -> 12).build()
    ),
    CORAL_GROUND_INTAKE(
        SuperstructureStateData.builder()
        .pose(Preset.CORAL_GROUND_INTAKE.getPose())
        .intakeVolts(() -> 12)
        .build()
    );

    private final SuperstructureStateData value;
} 