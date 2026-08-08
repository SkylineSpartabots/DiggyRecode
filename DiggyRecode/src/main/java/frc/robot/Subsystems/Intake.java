package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake subsystem — pulls balls from the floor into the robot.
 * Controlled via voltage output. Uses Brake mode to hold position when stopped.
 *
 * <p>
 * SysId routines are preserved in comments below for future characterization if
 * needed.
 */
public class Intake extends SubsystemBase {
    private static Intake instance;

    /**
     * Pre-allocated VoltageOut control request — reused to avoid per-call GC
     * allocation.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Returns the singleton instance of Intake, creating it if necessary.
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    /** Named voltage states for the intake motor. */
    public enum IntakeStates {
        ON(5),
        CYCLE(2),
        OFF(0),
        REVERSE(-2);

        double voltage;

        private IntakeStates(double voltage) {
            this.voltage = voltage;
        }

        /** Returns the voltage associated with this state (volts). */
        public double getVoltage() {
            return voltage;
        }
    }

    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(Constants.HardwarePorts.intake, "mechbussy");
        configureMotor(intakeMotor, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, stator current
     * limit,
     * a 50 Hz velocity signal update rate, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = direction;
        config.MotorOutput.NeutralMode = neutralMode;

        // Stator limit prevents the intake motor from burning out during hard jams
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.intakeStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Optimize CAN bus: this motor only runs open-loop, so we don't need fast feedback
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the intake motor output voltage.
     * Uses a pre-allocated VoltageOut to avoid object allocation on every call.
     *
     * @param voltage Voltage to apply (positive = intake direction)
     */
    public void setVoltage(double voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    /**
     * Returns a one-shot command that applies the given intake state.
     *
     * @param state The desired IntakeState (ON, CYCLE, OFF, REVERSE)
     * @return An InstantCommand-style command
     */
    public Command setState(IntakeStates state) {
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the intake at this time
    }

    // ---- SysId routines (preserved for future characterization, not currently
    // active) ----

    // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // Volts.of(1).div(Seconds.of(1)),
    // Volts.of(8),
    // Units.Seconds.of(8),
    // (state) -> SignalLogger.writeString("SysIdIntake_State", state.toString())
    // ),
    // new SysIdRoutine.Mechanism(
    // (volts) -> setVoltage(volts.in(Volts)),
    // log -> {log.motor("intake_motor")
    // .voltage(intakeMotor.getMotorVoltage().getValue())
    // .angularPosition(intakeMotor.getPosition().getValue())
    // .angularVelocity(intakeMotor.getVelocity().getValue()); },
    // this
    // ));

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.dynamic(direction);
    // }
}
