package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.HardwarePorts;

/**
 * Shooter subsystem — spins flywheel motors to launch balls at a target velocity.
 *
 * <p>Uses a 3-motor configuration (topL_leader, botL follower, topR opposed follower).
 * botR is wired but not currently used (slot commented out).
 *
 * <p>setVelocity(0) falls back to voltage=0 to ensure the motors coast down quietly
 * instead of fighting the PID trying to hold 0 rps.
 */
public class Shooter extends SubsystemBase {
    private static Shooter instance;

    /**
     * Returns the singleton instance of Shooter, creating it if necessary.
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /** Leader motor — top-left. All velocity commands go here; followers track it. */
    private TalonFX topL_leader;
    /** Bottom-left motor — follows topL_leader aligned (same direction). */
    private TalonFX botL;
    /** Top-right motor — follows topL_leader opposed (mirror direction). */
    private TalonFX topR;
    // private TalonFX botR; // bottom-right — wired but not installed this season

    /** Pre-allocated velocity control request. Slot 0 holds the PID + FF gains. */
    private final VelocityVoltage rpsRequest = new VelocityVoltage(0).withSlot(0);
    /** Pre-allocated voltage control request — used for coast-down and setVoltage(). */
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Cached leader velocity signal — cached once and refreshed via getValueAsDouble()
     * rather than calling topL_leader.getVelocity() each access, avoiding repeated
     * signal-handle lookups.
     */
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> leaderVelocitySignal;

    /** Stores the calculated ball air-time (seconds) for velocity-based lead calculations. */
    private double airtime;

    public Shooter() {
        topL_leader = new TalonFX(HardwarePorts.shooterTL, "mechbussy");
        botL        = new TalonFX(HardwarePorts.shooterBL, "mechbussy");
        topR        = new TalonFX(HardwarePorts.shooterTR, "mechbussy");
        // botR     = new TalonFX(HardwarePorts.shooterBR, "mechbussy");

        config(topL_leader, NeutralModeValue.Coast, InvertedValue.CounterClockwise_Positive);
        config(botL,        NeutralModeValue.Coast, InvertedValue.CounterClockwise_Positive);
        config(topR,        NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);
        // config(botR,     NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);

        // botL follows topL at aligned polarity (same direction)
        botL.setControl(new Follower(topL_leader.getDeviceID(), MotorAlignmentValue.Aligned));
        // topR follows topL at opposed polarity (mirror spin for ball grip on both sides)
        topR.setControl(new Follower(topL_leader.getDeviceID(), MotorAlignmentValue.Opposed));
        // botR.setControl(new Follower(topL_leader.getDeviceID(), MotorAlignmentValue.Opposed));

        // Followers don't need fast velocity updates to the Rio
        botL.getVelocity().setUpdateFrequency(4);
        topR.getVelocity().setUpdateFrequency(4);

        // Cache the leader velocity StatusSignal so we don't look it up on every call
        leaderVelocitySignal = topL_leader.getVelocity();
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, PID/FF gains,
     * current limits, and optimized CAN bus utilization.
     *
     * <p>The velocity signal update frequency is left at default (100 Hz) since the
     * shooter PID needs fast feedback. optimizeBusUtilization() suppresses all
     * other unused signals.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        // Velocity PID + feedforward gains (characterized via SysId)
        config.Slot0.kS = 0.3004;   // Static friction feedforward (V)
        config.Slot0.kV = 0.11613;  // Velocity feedforward (V / rps)
        config.Slot0.kA = 0.02206;  // Acceleration feedforward (V / rps²)
        config.Slot0.kP = 0.047173; // Proportional gain
        config.Slot0.kD = 0;        // Derivative gain

        // Supply + stator limits prevent brownouts and motor damage at high speeds
        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.shooterSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.shooterStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Suppress unused status signals to reduce CAN bus load.
        // Leader needs fast velocity for PID, but position is unused.
        motor.getPosition().setUpdateFrequency(4);
        
        motor.optimizeBusUtilization();
    }

    /**
     * Returns the shooter leader flywheel velocity in rotations-per-second (rps).
     * Uses the cached StatusSignal handle — no extra CAN lookup per call.
     *
     * @return Leader motor velocity in rps
     */
    public double getLeaderVelocity() {
        return leaderVelocitySignal.getValueAsDouble();
    }

    /**
     * Returns the velocities of all active flywheel motors in rps.
     * Order: [topL_leader, botL, topR].
     * botR is excluded as it is not installed this season.
     *
     * @return double array of velocities [topL, botL, topR]
     */
    public double[] getAllVelocities() {
        return new double[] {
            topL_leader.getVelocity().getValueAsDouble(),
            botL.getVelocity().getValueAsDouble(),
            topR.getVelocity().getValueAsDouble()
            // botR.getVelocity().getValueAsDouble() — not installed this season
        };
    }

    /**
     * Sets the shooter flywheel velocity using closed-loop PID + feedforward.
     * If velocity is 0, switches to voltage=0 so the motor coasts down instead
     * of the PID fighting to hold 0 rps (which causes stuttering).
     *
     * @param velocity Target velocity in rps
     */
    public void setVelocity(double velocity) {
        if (velocity == 0)
            topL_leader.setControl(voltageRequest.withOutput(0));
        else
            topL_leader.setControl(rpsRequest.withVelocity(velocity));
    }

    /**
     * Sets the shooter flywheel to an open-loop voltage output.
     * Used for coast-down, SysId routines, and emergency cases.
     *
     * @param voltage Voltage to apply (V)
     */
    public void setVoltage(double voltage) {
        topL_leader.setControl(voltageRequest.withOutput(voltage));
    }

    /**
     * Updates the stored ball air-time value (seconds).
     * Used by auto-aim calculations to lead the target based on robot velocity.
     *
     * @param airtime Estimated ball travel time in seconds
     */
    public void updateAirtime(double airtime) {
        this.airtime = airtime;
    }

    /**
     * Returns the last stored ball air-time in seconds.
     *
     * @return Ball air-time (seconds)
     */
    public double getAirtime() {
        return airtime;
    }

    // ---- SysId routines (used for shooter PID characterization) ----

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).div(Seconds.of(1)),  // Ramp rate: 1 V/s
            Volts.of(8),                      // Max dynamic step voltage
            Units.Seconds.of(8),              // Test timeout
            (state) -> SignalLogger.writeString("SysIdShooter_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> setVoltage(volts.in(Volts)),
            log -> {
                log.motor("shooter_leader")
                    .voltage(topL_leader.getMotorVoltage().getValue())
                    .angularPosition(topL_leader.getPosition().getValue())
                    .angularVelocity(topL_leader.getVelocity().getValue());
            },
            this
        )
    );

    /**
     * Returns a SysId quasistatic routine command for the shooter.
     * Used to characterize kS and kV feedforward gains.
     *
     * @param direction Forward or Reverse
     * @return SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a SysId dynamic routine command for the shooter.
     * Used to characterize kA feedforward gain.
     *
     * @param direction Forward or Reverse
     * @return SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}