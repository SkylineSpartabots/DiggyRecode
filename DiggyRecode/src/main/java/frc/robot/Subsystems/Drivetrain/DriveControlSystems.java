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

    /** Reusable request for auto-aim mode (heading locked to goal). */
    private final SwerveRequest.FieldCentricFacingAngle autoAimRequest =
            new SwerveRequest.FieldCentricFacingAngle();

    /** Reusable request for normal teleop drive (heading controlled by right stick). */
    private final SwerveRequest.FieldCentricFacingAngle driveRequest =
            new SwerveRequest.FieldCentricFacingAngle()
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
     * <p>If auto-aim is active, the heading is locked toward {@link #targetGoal} using
     * a ProfiledPID controller. Otherwise the right stick drives rotational rate directly.
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
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            driverLX *= -1;
            driverLY *= -1;
        }

        if (mode_AlignToGoal) {
            // Auto-aim: lock heading toward goal, use right-stick as feedforward
            return autoAimRequest
                    .withVelocityX(driverLY)
                    .withVelocityY(driverLX)
                    .withTargetRateFeedforward(calculateGoalHeading());
        }

        // Normal teleop drive — right stick controls rotation rate
        return driveRequest
                .withVelocityX(driverLY)
                .withVelocityY(driverLX)
                .withTargetRateFeedforward(driverRX);

        // Alternative robot-centric mode (commented out):
        // return new SwerveRequest.RobotCentric()
        //     .withVelocityX(driverLY)
        //     .withVelocityY(driverLX)
        //     .withRotationalRate(driverRX);
    }

    /**
     * Calculates the desired rotational velocity feedforward to face the target goal.
     * Uses a ProfiledPID controller on heading error and a distance-based FF scaler.
     *
     * @return Rotational rate feedforward (rad/s), or 0.0 if already at goal heading
     */
    private double calculateGoalHeading() {
        var state = s_Swerve.getState();

        double currentDistance = state.Pose.getTranslation().getDistance(targetGoal);

        // Simple heading angle toward target (no velocity lead — lead calculation is commented below)
        targetHeading = Math.atan2(
            (targetGoal.getY() - state.Pose.getY()),
            (targetGoal.getX() - state.Pose.getX()));

        // Velocity-lead heading (preserved for future use with airtime compensation):
        // double airtime = s_Shooter.getAirtime();
        // ChassisSpeeds velocityOffset = state.Speeds.times(airtime);
        // targetHeading = Math.atan2(
        //     (velocityOffset.vyMetersPerSecond + targetGoal.getY() - state.Pose.getY()),
        //     (velocityOffset.vxMetersPerSecond + targetGoal.getX() - state.Pose.getX()));

        // Scale feedforward from 0 at ffMinRadius to 1 at ffMaxRadius
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0, 1.0);

        // Pre-allocate goal state to avoid WPILib allocating a new State(double, 0) internally
        autoAimGoalState.position = targetHeading;
        autoAimGoalState.velocity = 0;

        // Combine profiled PID output with distance-weighted velocity feedforward
        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(state.Pose.getRotation().getRadians(), autoAimGoalState);

        return thetaController.atGoal() ? 0 : thetaVelocity;
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
        mode_AlignToGoal = true;
        targetGoal = DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? Constants.FieldConstants.blueGoal.toTranslation2d()
                : Constants.FieldConstants.redGoal.toTranslation2d();
    }

    /**
     * Disables auto-aim mode. The right stick resumes controlling rotation directly.
     */
    public void turnOffAutoAim() {
        mode_AlignToGoal = false;
    }
}
