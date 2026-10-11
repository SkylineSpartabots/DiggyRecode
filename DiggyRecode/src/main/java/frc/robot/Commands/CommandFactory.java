package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.Automation.AlignToGoal;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Shooter.RampShooterWithDistance;
import frc.robot.Commands.Shooter.SetShooter;
import frc.robot.Commands.Shooter.SetShooterAtMeter;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.DriveControlSystems;
import frc.robot.Subsystems.Conveyor.ConveyorStates;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Intake.IntakeStates;

/**
 * CommandFactory — static factory methods for composing multi-subsystem command sequences.
 *
 * <p>Centralizing complex command compositions here keeps RobotContainer clean
 * and makes it easy to adjust timing/sequencing in one place.
 */
public class CommandFactory {

    /**
     * End-of-auto shot. Ramps from odometry, turns in place toward the hub, and feeds
     * once the wheel is up to speed and the robot is facing it.
     * Holds the drivetrain until the auto command is cancelled.
     */
    public static Command AutoAimShoot() {
        return new RampShooterWithDistance().alongWith(
            new AlignToGoal(() -> 0, () -> 0),
            feedWhenAimed()
        );
    }

    /**
     * Teleop shot. Ramps from odometry and feeds once the wheel is at speed and aim
     * reports the robot is facing the hub. Does not take the drivetrain; schedule
     * {@link AlignToGoal} next to this so the left stick can still translate.
     */
    public static Command DriverShoot() {
        return new RampShooterWithDistance().alongWith(feedWhenAimed());
    }

    /**
     * Ramps the shooter from odometry and feeds once measured speed is close to the request.
     * No aiming, so a path command can own the drivetrain at the same time.
     */
    public static Command ShootAtDistance() {
        return new RampShooterWithDistance().alongWith(feedWhenShooterReady());
    }

    /**
     * Shooter only. Spins up from odometry and does not feed or aim.
     */
    public static Command RampShooter() {
        return new RampShooterWithDistance();
    }

    /**
     * Indexer and conveyor only. Driver button for feeding. Does not touch the shooter.
     */
    public static Command Feed() {
        return new ParallelCommandGroup(
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    /**
     * Waits until the flywheel is within tolerance of the odometry request, then feeds.
     * Times out so auto cannot sit forever if the wheel never gets there.
     */
    private static Command feedWhenShooterReady() {
        return Commands.waitUntil(() -> Shooter.getInstance().isReadyToFeed(Constants.shooterFeedToleranceRps))
            .withTimeout(Constants.shooterFeedTimeoutSec)
            .andThen(Feed());
    }

    /**
     * Feeds once the flywheel is at the distance speed and the robot is facing the hub.
     * If the driver takes rotation back (right trigger), the wheel-speed check is enough.
     * The timeout still feeds if aim or the wheel never gets there.
     */
    private static Command feedWhenAimed() {
        return Commands.waitUntil(() -> {
            boolean wheelReady = Shooter.getInstance().isReadyToFeed(Constants.shooterFeedToleranceRps);
            DriveControlSystems aim = DriveControlSystems.getInstance();
            boolean pointing = !aim.isAutoAiming() || aim.isAimed();
            return wheelReady && pointing;
        }).withTimeout(Constants.shooterFeedTimeoutSec).andThen(Feed());
    }

    /**
     * Stops all active mechanisms — indexer, conveyor, intake, and shooter.
     * Use this as a safe "kill switch" for the scoring system.
     *
     * @return Sequential command that turns off all mechanisms in order
     */
    public static Command AllOff() {
        return new SequentialCommandGroup(
            new SetIndexer(IndexerStates.OFF),
            new SetConveyor(ConveyorStates.OFF),
            new SetIntake(IntakeStates.OFF),
            new SetShooter(0)
        );
    }

    /**
     * Activates the intake to pull balls onto the robot.
     *
     * @return Command that sets intake to ON state
     */
    public static Command IntakeBallsON() {
        return new SequentialCommandGroup(
            new SetIntake(IntakeStates.ON)
        );
    }

    /**
     * Deactivates the intake.
     *
     * @return Command that sets intake to OFF state
     */
    public static Command IntakeBallsOFF() {
        return new SequentialCommandGroup(
            new SetIntake(IntakeStates.OFF)
        );
    }

    /**
     * Shoots balls at a fixed distance. The shooter still slews at 10 rps/s, so feeding
     * waits until measured speed is close instead of a fixed 1 second.
     *
     * @param distance Target distance in meters
     */
    public static Command LobAtMeter(double distance) {
        return new SequentialCommandGroup(
            new SetShooterAtMeter(distance),
            Commands.waitUntil(() -> Shooter.getInstance().isReadyToFeed(Constants.shooterFeedToleranceRps))
                .withTimeout(Constants.shooterFeedTimeoutSec),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    /**
     * Shoots balls at a fixed explicit RPS target, then feeds after the ramp has had time to arrive.
     * Useful for testing a specific shooter speed without distance math.
     *
     * @param rps Target flywheel speed in rotations-per-second
     * @return Sequential command: set shooter RPS → wait out the ramp → feed balls
     */
    public static Command LobAtRps(double rps) {
        // 10 rps/s, plus a quarter second for the wheel to catch the setpoint.
        double rampSeconds = Math.abs(rps) / Constants.shooterRampRpsPerSec + 0.25;
        return new SequentialCommandGroup(
            new SetShooter(rps),
            new WaitCommand(rampSeconds),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    /**
     * Runs conveyor and indexer in reverse to spit balls back out.
     * THIS IS THE REVERSE STATES
     * @return Parallel command setting both mechanisms to REVERSE
     */
    public static Command ReverseFeed() {
        return new ParallelCommandGroup(
            new SetConveyor(ConveyorStates.REVERSE),
            new SetIndexer(IndexerStates.REVERSE)
        );
    }
}
