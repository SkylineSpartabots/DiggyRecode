package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climb subsystem — drives two climb motors (leader + follower) to extend/retract the climber.
 *
 * <p><b>NOTE:</b> This subsystem is NOT bound to any controller buttons this season.
 * The code is fully preserved for future use. To re-enable, uncomment the Climb
 * instantiation in {@link frc.robot.RobotContainer} and {@link frc.robot.Robot}.
 *
 * <p>Uses percent output (duty cycle) control via {@link TalonFX#set(double)}.
 */
public class Climb extends SubsystemBase {
    private static Climb instance;

    /**
     * Returns the singleton instance of Climb, creating it if necessary.
     * Even though Climb is not bound this season, keeping it as a singleton
     * prevents multiple hardware instances if code is accidentally called.
     */
    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    /**
     * Named speed states for the climb motors.
     * Positive = climb up (extend), negative = reverse/retract.
     */
    public enum ClimbStates {
        ON(0.4),
        OFF(0),
        REVERSE(-0.4);

        double speed;
        private ClimbStates(double speed) {
            this.speed = speed;
        }

        /** Returns the percent output associated with this state [-1, 1]. */
        public double getSpeed() {
            return speed;
        }
    }

    /** Leader motor (left climb). All commands go here. */
    private final TalonFX climbMotorLeader;
    /** Follower motor (right climb). Mirrors the leader at opposed polarity. */
    private final TalonFX climbMotorFollower;

    public Climb() {
        climbMotorLeader   = new TalonFX(Constants.HardwarePorts.climbL, "mechbussy");
        climbMotorFollower = new TalonFX(Constants.HardwarePorts.climbR, "mechbussy");

        config(climbMotorLeader,   NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
        config(climbMotorFollower, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

        // follower opposes leader so both sides pull in the same physical direction
        climbMotorFollower.setControl(new Follower(climbMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, a 50 Hz velocity
     * signal rate, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        motor.getConfigurator().apply(config);

        // Optimize CAN bus: open-loop control doesn't need fast position/velocity updates
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the climb motor leader to a raw percent output.
     * The follower tracks automatically.
     *
     * @param speed Percent output [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        climbMotorLeader.set(speed);
    }

    /**
     * Returns a one-shot command that applies the given climb state.
     * Not currently bound to any controller — preserved for future seasons.
     *
     * @param state The desired ClimbState (ON, OFF, REVERSE)
     * @return An InstantCommand-style command
     */
    public Command setState(ClimbStates state) {
        return Commands.runOnce(() -> setSpeed(state.getSpeed()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the climber at this time
    }
}
