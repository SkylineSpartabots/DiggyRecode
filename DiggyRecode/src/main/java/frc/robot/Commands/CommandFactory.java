package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Shooter.RampShooterWithDistance;
import frc.robot.Commands.Shooter.SetShooter;
import frc.robot.Commands.Shooter.SetShooterAtMeter;
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
     * Aligns to the goal using vision (currently just waits 1 second),
     * then feeds balls into the shooter using the pre-ramped distance-based velocity.
     *
     * @return Sequential command: wait for alignment → index + convey, alongside distance ramp
     */
    public static Command AutoAimShoot() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // new AlignToGoal(), // vision auto-aim — re-enable when tuned
                new WaitCommand(1)
            ),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        ).alongWith(new RampShooterWithDistance());
    }

    /**
     * Waits 1 second for the shooter to ramp up, then feeds balls through indexer + conveyor.
     * The shooter ramps to a computed distance-based RPS in parallel.
     *
     * @return Sequential command: 1s wait → feed balls, alongside distance-based shooter ramp
     */
    public static Command ShootAtDistance() {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        ).alongWith(new RampShooterWithDistance());
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
     * Shoots balls at a fixed distance by computing the required RPS from a lookup,
     * waiting 1 second to ramp up, then feeding.
     *
     * @param distance Target distance in meters
     * @return Sequential command: set shooter velocity → wait → feed balls
     */
    public static Command LobAtMeter(double distance) {
        return new SequentialCommandGroup(
            new SetShooterAtMeter(distance),
            new WaitCommand(1),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    /**
     * Shoots balls at a fixed explicit RPS target, then feeds after a 1.25s ramp delay.
     * Useful for testing a specific shooter speed without distance math.
     *
     * @param rps Target flywheel speed in rotations-per-second
     * @return Sequential command: set shooter RPS → 1.25s wait → feed balls
     */
    public static Command LobAtRps(double rps) {
        return new SequentialCommandGroup(
            new SetShooter(rps),
            new WaitCommand(1.25),
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
