package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter;

/**
 * DriveControlSystems — wraps swerve drive input processing and auto-aim logic.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>Applies deadband + cubic scaling to joystick inputs</li>
 *   <li>Flips field-relative translation for red alliance</li>
 *   <li>Provides auto-aim heading control using a ProfiledPID targeting the goal</li>
 * </ul>
 *
 * <p>SwerveRequest objects are pre-allocated as fields to avoid heap allocation
 * every robot loop (which would cause CAN bus timing jitter and GC stutters).
 */
public class DriveControlSystems {

    /** Controls how "linear" the joystick feels. Higher = more linear, lower = more cubic. */
    private double deadbandFactor = 0.85;

    private static CommandSwerveDrivetrain s_Swerve;

    /**
     * Profiled PID controller for heading auto-aim.
     * Uses a trapezoidal profile to limit angular acceleration when snapping to target.
     */
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3, 0, 0.02,
            new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity, Constants.MaxAngularRate),
            0.02);

    /** Whether auto-aim (heading lock toward goal) is active. */
    boolean mode_AlignToGoal = false;
    /** False until goalRotationRate() has run since the last turn-on, so a stale atGoal() cannot feed. */
    private boolean aimSettled = false;
    /** The field-relative translation of the target goal (blue or red, set on mode change). */
    Translation2d targetGoal;
    /** Most recently calculated desired heading (radians). */
    double targetHeading;
    /** Feedforward scaling radius bounds — scales the theta FF based on distance to target. */
    double ffMinRadius = 0.2, ffMaxRadius = 1.2;

    private static DriveControlSystems instance;

    /**
     * Returns the singleton instance of DriveControlSystems, creating it if necessary.
     */
    public static DriveControlSystems getInstance() {
        if (instance == null) {
            instance = new DriveControlSystems();
        }
        return instance;
    }

    /** Pre-allocated goal state to prevent ProfiledPIDController.calculate(double, double) from allocating memory. */
    private final TrapezoidProfile.State autoAimGoalState = new TrapezoidProfile.State();

    // ---- Pre-allocated SwerveRequest objects ----
    // Avoids creating new objects each robot loop (20ms) which causes GC pauses
    // and can introduce CAN timing jitter.

    // Both requests use the blue-alliance field frame and drive() flips translation on red.
    // Auto-aim's heading math is in that same frame, so it must not be left on the default
    // OperatorPerspective, or the two modes would disagree the moment a perspective is set.

    /**
     * One request for both manual drive and auto-aim. Auto-aim puts the profiled
     * heading rate in withRotationalRate. FieldCentricFacingAngle was the wrong
     * request: its feedforward is added on top of a heading controller that was
     * still aimed at 0°.
     */
    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withDesaturateWheelSpeeds(true)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public DriveControlSystems() {
        s_Swerve  = CommandSwerveDrivetrain.getInstance();

        thetaController.setTolerance(Math.toRadians(4));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // =======---===[ ⚙ Joystick processing ]===---========

    /**
     * Builds the swerve drive request for the current loop based on joystick inputs.
     * Applies deadband + cubic scaling before mapping to chassis speeds.
     *
     * <p>If auto-aim is active, rotation faces {@link #targetGoal} and the right stick
     * is ignored. Otherwise the right stick drives rotational rate directly.
     *
     * @param driverLY  Left stick Y axis (raw [-1, 1]) — translates forward/back
     * @param driverLX  Left stick X axis (raw [-1, 1]) — translates left/right
     * @param driverRX  Right stick X axis (raw [-1, 1]) — rotates when not auto-aiming
     * @return The SwerveRequest to pass to {@link CommandSwerveDrivetrain#applyRequest}
     */
    public SwerveRequest drive(double driverLY, double driverLX, double driverRX) {
        // Scale joystick inputs through deadband + cubic curve → physical units
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxAngularRate;

        // Flip translation axes on red alliance so driver always faces "forward"
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            driverLX *= -1;
            driverLY *= -1;
        }

        if (mode_AlignToGoal) {
            return driveRequest
                    .withVelocityX(driverLY)
                    .withVelocityY(driverLX)
                    .withRotationalRate(goalRotationRate());
        }

        // Normal teleop drive — right stick controls rotation rate
        return driveRequest
                .withVelocityX(driverLY)
                .withVelocityY(driverLX)
                .withRotationalRate(driverRX);

        // Alternative robot-centric mode (commented out):
        // return new SwerveRequest.RobotCentric()
        //     .withVelocityX(driverLY)
        //     .withVelocityY(driverLX)
        //     .withRotationalRate(driverRX);
    }

    /**
     * Rotational rate (rad/s) that points the robot's front at {@link #targetGoal}.
     * Zero when the heading is already inside tolerance, or before a target exists.
     */
    private double goalRotationRate() {
        if (targetGoal == null) {
            return 0;
        }

        var state = s_Swerve.getState();
        double currentDistance = state.Pose.getTranslation().getDistance(targetGoal);

        // Field heading from the robot to the hub. +X on the robot is forward.
        targetHeading = Math.atan2(
            (targetGoal.getY() - state.Pose.getY()),
            (targetGoal.getX() - state.Pose.getX()));

        // Scale the profile's cruise velocity down when the robot is close to the hub.
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0, 1.0);

        autoAimGoalState.position = targetHeading;
        autoAimGoalState.velocity = 0;

        // calculate() updates the profile. The setpoint velocity has to be read after that.
        double correction = thetaController.calculate(
                state.Pose.getRotation().getRadians(), autoAimGoalState);
        aimSettled = true;

        if (thetaController.atGoal()) {
            return 0;
        }
        return thetaController.getSetpoint().velocity * ffScaler + correction;
    }

    /** True while a shoot or the left trigger is holding heading on the hub. */
    public boolean isAutoAiming() {
        return mode_AlignToGoal;
    }

    /**
     * True once this aim cycle has run and the robot is inside the heading tolerance.
     * False on the first loop after turning aim on, so a leftover atGoal() cannot feed.
     */
    public boolean isAimed() {
        return mode_AlignToGoal && aimSettled && thetaController.atGoal();
    }

    /**
     * Applies a deadband and cubic scaling curve to a joystick axis input.
     *
     * <p>Formula: {@code deadbandFactor * x³ + (1 - deadbandFactor) * x}<br>
     * This blends linear and cubic responses for smooth low-speed control
     * while still reaching full speed at the stick limits.
     *
     * @param input Raw joystick input [-1, 1]
     * @return Scaled output [-1, 1], or 0 if within deadband
     */
    private double scaledDeadBand(double input) {
        if (Math.abs(input) < Constants.stickDeadband)
            return 0;
        else
            return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    // ---- Mode switching ----

    /**
     * Enables auto-aim mode. The drivetrain will rotate toward the alliance goal
     * (blue or red depending on DS alliance setting) while the driver translates normally.
     */
    public void turnOnAutoAim() {
        targetGoal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Constants.FieldConstants.blueGoal.toTranslation2d()
                : Constants.FieldConstants.redGoal.toTranslation2d();

        // Start the profile from where the robot is now. Otherwise it resumes from the
        // last setpoint (or 0 rad on first use) and whips toward that before the goal.
        var state = s_Swerve.getState();
        thetaController.reset(state.Pose.getRotation().getRadians(), state.Speeds.omegaRadiansPerSecond);
        aimSettled = false;
        mode_AlignToGoal = true;
    }

    /**
     * Disables auto-aim mode. The right stick resumes controlling rotation directly.
     */
    public void turnOffAutoAim() {
        mode_AlignToGoal = false;
        aimSettled = false;
    }
}
