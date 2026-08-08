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
 * Conveyor subsystem — moves balls from the intake zone into the indexer.
 * Controlled via voltage. Runs in Coast mode so balls don't jam on stop.
 */
public class Conveyor extends SubsystemBase {
    private static Conveyor instance;

    /**
     * Returns the singleton instance of Conveyor, creating it if necessary.
     */
    public static Conveyor getInstance() {
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    /**
     * Pre-allocated VoltageOut control request — reused to avoid per-call GC
     * allocation.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /** Named voltage states for the conveyor. */
    public enum ConveyorStates {
        ON(4.5),
        CYCLE(3),
        OFF(0),
        REVERSE(-3);

        double voltage;

        private ConveyorStates(double voltage) {
            this.voltage = voltage;
        }

        /** Returns the voltage associated with this state (volts). */
        public double getVoltage() {
            return voltage;
        }
    }

    private final TalonFX conveyorMotor;

    public Conveyor() {
        conveyorMotor = new TalonFX(Constants.HardwarePorts.conveyor, "mechbussy");
        config(conveyorMotor, NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, current limits,
     * a 50 Hz velocity signal update rate, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        // Supply + stator current limits to protect motor and PDP
        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.conveyorSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.conveyorStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Optimize CAN bus: this motor only runs open-loop, so we don't need fast feedback
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the conveyor motor output voltage directly.
     *
     * @param voltage Voltage to apply (positive = toward indexer)
     */
    public void setVoltage(double voltage) {
        conveyorMotor.setControl(voltageRequest.withOutput(voltage));
    }

    /**
     * Returns a one-shot command that applies the given conveyor state.
     *
     * @param state The desired ConveyorState
     * @return An InstantCommand-style command
     */
    public Command setState(ConveyorStates state) {
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the conveyor at this time
    }
}
