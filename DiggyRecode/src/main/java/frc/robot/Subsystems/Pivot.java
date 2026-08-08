package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Pivot subsystem — controls the shooter angle relative to the robot.
 * Currently uses open-loop voltage control. MotionMagic position control
 * is stubbed out in comments for future use.
 *
 * <p>Uses Arm_Cosine gravity compensation type so kG properly counteracts
 * gravity at all pivot angles.
 */
public class Pivot extends SubsystemBase {
    private static Pivot instance;

    /** Pre-allocated VoltageOut control request — reused to avoid GC allocation per call. */
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Returns the singleton instance of Pivot, creating it if necessary.
     */
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    /**
     * Named voltage states for the pivot.
     * Negative voltage = deploying (pivoting outward/down), positive = stowing.
     */
    public enum PivotStates {
        DEPLOYED(-4),
        MIDDLE(-2), // TODO: tune for actual midpoint
        STOWED(0);

        double volts;
        private PivotStates(double volts) {
            this.volts = volts;
        }

        /** Returns the voltage associated with this pivot state. */
        public double getVolts() {
            return volts;
        }
    }

    private final TalonFX pivotMotor;

    // private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    // ^ MotionMagic position control — reserved for when encoder-based control is tuned

    public Pivot() {
        pivotMotor = new TalonFX(Constants.HardwarePorts.pivot, "mechbussy");
        configureMotor(pivotMotor, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, current limits,
     * arm-cosine gravity compensation, a 50 Hz velocity signal rate, and
     * optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Arm_Cosine makes kG correct for arm-style mechanisms (gravity effect varies with angle)
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // PID + feedforward gains (commented out until MotionMagic is commissioned):
        // config.Slot0.kP = 0.01;
        // config.Slot0.kD = 0.01;
        // config.Slot0.kG = 0.5;

        config.MotorOutput.Inverted = direction;
        config.MotorOutput.NeutralMode = neutralMode;

        // Supply + stator current limits to protect the pivot mechanism
        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.pivotSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.pivotStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Zero the encoder position at startup (assume mechanism starts at stow)
        motor.setPosition(0);

        // Optimize CAN bus: open-loop control doesn't need fast position/velocity updates
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    // public void setRotations(double rotations) {
    //     pivotMotor.setControl(mmRequest.withPosition(rotations));
    // }
    // ^ MotionMagic position setter — activate once PID gains are tuned

    /**
     * Sets the pivot motor output voltage directly (open-loop control).
     * Uses a pre-allocated VoltageOut to avoid object allocation on each call.
     *
     * @param volts Voltage to apply (negative = deploy, 0 = hold/stow)
     */
    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Sets the pivot motor to a raw percent output (0.0–1.0).
     * Use sparingly; prefer {@link #setVoltage(double)} for more predictable behavior.
     *
     * @param speed Percent output [-1, 1]
     */
    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * Returns a one-shot command that applies the given pivot state.
     *
     * @param state The desired PivotState (DEPLOYED, MIDDLE, STOWED)
     * @return An InstantCommand-style command
     */
    public Command setState(PivotStates state) {
        return Commands.runOnce(() -> setVoltage(state.getVolts()), this);
    }

    /**
     * Zeroes the pivot encoder. Call when the mechanism is at its known stow position.
     */
    public void zeroPivot() {
        pivotMotor.setPosition(0);
    }

    /**
     * Returns the raw stator current signal from the pivot motor.
     * Used externally to detect stall/jam conditions.
     *
     * @return StatusSignal for pivot stator current (amps)
     */
    public StatusSignal<Current> getCurrent() {
        return pivotMotor.getStatorCurrent();
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the pivot at this time
    }
}
