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
 * Indexer subsystem — feeds balls from the conveyor into the shooter.
 * Controlled via voltage output; uses a pre-allocated VoltageOut request to
 * avoid per-call object allocation and reduce GC pressure.
 */
public class Indexer extends SubsystemBase {
    private static Indexer instance;

    /**
     * Returns the singleton instance of Indexer, creating it if necessary.
     * All subsystems use singletons so hardware is only configured once.
     */
    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    /** Named voltage states for the indexer. Positive = toward shooter. */
    public enum IndexerStates {
        ON(6),
        OFF(0),
        REVERSE(-3);

        double voltage;

        private IndexerStates(double voltage) {
            this.voltage = voltage;
        }

        /** Returns the voltage associated with this state. */
        public double getVoltage() {
            return voltage;
        }
    }

    private final TalonFX indexerMotor;

    /**
     * Pre-allocated VoltageOut control request.
     * Reusing this object avoids creating garbage on every setVoltage() call.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Indexer() {
        indexerMotor = new TalonFX(Constants.HardwarePorts.indexer, "mechbussy");
        config(indexerMotor, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, direction, current limits,
     * a reduced velocity signal frequency (50 Hz = 20 ms), and optimized CAN bus
     * utilization (suppresses status frames that aren't being read).
     *
     * @param motor       The TalonFX motor to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        // Stator current limit protects the motor from overheating under load
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.indexerStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply config once (previously applied twice — bug fixed)
        motor.getConfigurator().apply(config);

        // Optimize CAN bus: this motor only runs open-loop, so we don't need fast feedback
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress all unused status signals to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the indexer motor output voltage directly.
     * Uses a pre-allocated VoltageOut request to avoid heap allocation per call.
     *
     * @param voltage Voltage to apply (positive = toward shooter)
     */
    public void setVoltage(double voltage) {
        indexerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    /**
     * Returns a one-shot command that sets the indexer to the given state.
     *
     * @param state The desired IndexerState (ON, OFF, REVERSE)
     * @return An InstantCommand-style command that applies the voltage
     */
    public Command setState(IndexerStates state) {
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the indexer at this time
    }
}
