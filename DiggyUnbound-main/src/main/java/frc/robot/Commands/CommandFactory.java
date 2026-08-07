package frc.robot.Commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.SetPivot;
import frc.robot.Commands.Shooter.RampShooterWithDistance;
import frc.robot.Commands.Shooter.SetShooter;
import frc.robot.Commands.Shooter.SetShooterAtMeter;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Conveyor.ConveyorStates;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

public class CommandFactory {

// intake --------------------------

    public static Command IntakeBallsON(){
        return new SequentialCommandGroup(
            new SetIntake(IntakeStates.ON)
        );
    }

    public static Command IntakeBallsOFF(){
        return new SequentialCommandGroup(
            new SetIntake(IntakeStates.OFF)
        );
    }

// shooter ----------------------------

    public static Command LobAtMeter(double distance){
        return new SequentialCommandGroup(
            new SetShooterAtMeter(distance),
            new WaitCommand(1),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    public static Command LobAtRps(double rps){
        return new SequentialCommandGroup(
            new SetShooter(rps),
            new WaitCommand(1.25),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

    public static Command AutoAimShoot(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // new AlignToGoal(),
                new WaitCommand(1)
            ),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        ).alongWith(new RampShooterWithDistance());
    }

    public static Command ShootAtDistance(){
        return new SequentialCommandGroup(
            new WaitCommand(1),
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        ).alongWith(new RampShooterWithDistance());
    }

    public static Command chud(){
        return new SequentialCommandGroup(
            new SetIndexer(IndexerStates.ON),
            new SetConveyor(ConveyorStates.ON)
        );
    }

// misc ---------------------------------

    public static Command reverseStates(){
        return new ParallelCommandGroup(
            new SetIndexer(IndexerStates.REVERSE),
            new SetConveyor(ConveyorStates.REVERSE)
        );
    }

    public static Command AllOff(){
        return new SequentialCommandGroup(
            new SetIndexer(IndexerStates.OFF),
            new SetConveyor(ConveyorStates.OFF),
            new SetPivot(PivotStates.OFF),
            new SetIntake(IntakeStates.OFF),
            new SetShooter(0)
        );
    }
}
