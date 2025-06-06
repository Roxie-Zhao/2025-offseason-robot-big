package frc.robot.subsystems.superstructure.endeffectorarm;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.EndEffectorArmConstants;

import static edu.wpi.first.units.Units.Volts;

public class EndEffectorArmPivotIOSim implements EndEffectorArmPivotIO {
    // Default values - adjust according to your specific mechanism
    private static final double moi = 1.0; // Moment of inertia
    private static final double cgRadius = 0.2; // Center of gravity radius
    private static final double PIVOT_RATIO = 50.0; // Default gear ratio

    private static final DCMotor gearbox =
            DCMotor.getKrakenX60Foc(1).withReduction(PIVOT_RATIO);

    // Calculate scale factor from motor parameters
    private static final double SCALE_FACTOR = (gearbox.KtNMPerAmp / moi) 
                                             * (1.0 / gearbox.rOhms)
                                             * 0.35; // Damping factor to reduce response

    private static final Matrix<N2, N2> A =
            MatBuilder.fill(
                    Nat.N2(),
                    Nat.N2(),
                    0,
                    1,
                    0,
                    -gearbox.KtNMPerAmp
                            / (gearbox.KvRadPerSecPerVolt
                            * gearbox.rOhms * moi));
    private static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

    private final ProfiledPIDController controller = new ProfiledPIDController(
            EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KP.get(),
            EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KI.get(),
            EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KD.get(),
            new Constraints(
                180.0, // deg/s, adjust as needed
                180  // deg/s^2, adjust as needed
            ));

    // Simulation state
    private Vector<N2> simState;
    private Measure<VoltageUnit> appliedVolts = Volts.zero();
    private double inputTorqueCurrent = 0.0;
    private double setpointDegrees = 0.0;

    public EndEffectorArmPivotIOSim() {
        // Initial position (90 degrees)
        simState = VecBuilder.fill(Math.PI / 2.0, 0.0);
    }

    @Override
    public void updateInputs(EndEffectorArmPivotIOInputs inputs) {
        for (int i = 0; i < RobotConstants.LOOPER_DT / (1.0 / 1000.0); i++) {
            double feedforward = EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KS.get() * Math.signum(simState.get(1)) +
                               EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KV.get() * simState.get(1) +
                               EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KA.get() * 0.0 + // acceleration term
                               EndEffectorArmConstants.EndEffectorArmPivotGainsClass.END_EFFECTOR_ARM_PIVOT_KG.get(); // gravity compensation
            setInputTorqueCurrent(
                    controller.calculate(simState.get(0)) * SCALE_FACTOR + feedforward);
            update(1.0 / 1000.0);
        }
        inputs.targetAngleDeg = setpointDegrees;
        inputs.currentAngleDeg = Math.toDegrees(simState.get(0));
        inputs.velocityRotPerSec = Math.toDegrees(simState.get(1));
        inputs.appliedVolts = appliedVolts.magnitude();
        inputs.statorCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.tempCelsius = 20.0; // Default room temperature
    }

    @Override
    public void setPivotAngle(double targetAngleDeg) {
        this.setpointDegrees = targetAngleDeg;
        controller.setGoal(Math.toRadians(targetAngleDeg));
    }

    @Override
    public void resetAngle(double resetAngleDeg) {
        // Reset simulation angle
        simState = VecBuilder.fill(Math.toRadians(resetAngleDeg), 0.0);
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts =
                Volts.of(gearbox.getVoltage(
                        gearbox.getTorque(inputTorqueCurrent),
                        simState.get(1, 0)));
        appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.magnitude(), -12.0, 12.0));
    }

    private void update(double dt) {
        inputTorqueCurrent =
                MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);

        Matrix<N2, N1> updatedState =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                                A.times(x)
                                        .plus(B.times(u)),
                        simState,
                        VecBuilder.fill(inputTorqueCurrent * SCALE_FACTOR),
                        dt);
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
} 