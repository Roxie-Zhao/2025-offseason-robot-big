package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.superstructure.SuperstructureState;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.auto.AutoActions.*;

public class AutoLeft5C1A extends AutoRoutine {
  private static final Pose2d startPose = new Pose2d(
      new Translation2d(7.140, FieldConstants.fieldWidth - 0.50),
      Rotation2d.kZero
  );

  public AutoLeft5C1A() {
    super("Left5C1A");
  }

  @Override
  public Command getAutoCommand() {
    var scorePreload = sequence(
        setGoal(AimGoalSupplier.ReefFace.FarLeftTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake1 = deadline(
        driveToDecisionPoint(true, true),
        intake()
    );

    var scoreNearL4Right = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake2 = deadline(
        driveToDecisionPoint(true, false),
        intake()
    );

    var scoreNearL4Left = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L4),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake3 = deadline(
        driveToDecisionPoint(true, false),
        intake()
    );

    var scoreNearL3Right = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, true, SuperstructureState.L3),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var driveToDpAndIntake4 = deadline(
        driveToDecisionPoint(true, false),
        intake()
    );

    var scoreNearL3Left = sequence(
        setGoal(AimGoalSupplier.ReefFace.NearLeftTilt, false, SuperstructureState.L3),
        parallel(
            driveToSelectedTarget(),
            prepare()
        ),
        shoot()
    );

    var ending = sequence(
        takeAlgae(),
        driveToEndPoint(true),
        indicateEnd()
    );


    return sequence(
        scorePreload,
        driveToDpAndIntake1,
        scoreNearL4Right,
        driveToDpAndIntake2,
        scoreNearL4Left,
        driveToDpAndIntake3,
        scoreNearL3Right,
        driveToDpAndIntake4,
        scoreNearL3Left,
        ending
    );
  }

  @Override
  public Command getOnSelectCommand() {
    return resetOnPose(startPose);
  }
}
