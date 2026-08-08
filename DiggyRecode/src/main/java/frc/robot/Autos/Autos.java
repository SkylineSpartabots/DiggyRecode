// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CommandFactory;
import frc.robot.Commands.Automation.JiggleBallsDrivetrainNoRolla;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.ForcePivot;
import frc.robot.Subsystems.Intake.IntakeStates;

public final class Autos {
    static Timer path_time = new Timer();

    public static Command getAutoCommand(AutoPath autoPath) {
        return autoPath.autoCommand;
    }

    public static Command depo_simple() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("depo_simple");
        return new SequentialCommandGroup(
                new ForcePivot(),
                CommandFactory.IntakeBallsON(),
                new SequentialCommandGroup(
                        new WaitCommand(3.3),
                        CommandFactory.IntakeBallsOFF()).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.AutoAimShoot().raceWith(new WaitCommand(6))
            );
    }

    public static Command mid() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid");
        return new SequentialCommandGroup(
                new WaitCommand(2.5),
                new ForcePivot(),
                CommandFactory.ShootAtDistance()
        ).alongWith(new FollowChoreoTrajectory(traj));
    }

    public static Command mid_to_depo() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid_to_depo");
        return new SequentialCommandGroup(
                new WaitCommand(2),
                new ForcePivot(), //2.5
                new SetIntake(IntakeStates.ON),
                new WaitCommand(3), //5.5
                CommandFactory.ShootAtDistance()
        ).alongWith(new FollowChoreoTrajectory(traj));
    }

    public static Command mid_right() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid_right");
        return new SequentialCommandGroup(
                new WaitCommand(2.25),
                new ForcePivot(),
                CommandFactory.ShootAtDistance()
        ).alongWith(new FollowChoreoTrajectory(traj));
    }

    public static Command mid_tuning() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid_tuning");
        return new SequentialCommandGroup(
            new FollowChoreoTrajectory(traj)
            // CommandFactory.IntakeBallsON(),
            // new WaitCommand(1),
            // CommandFactory.IntakeBallsOFF()
        );


    }

    public static Command trench_right_right_mid_chill() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_right_right_mid_chill");
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.65),
                new ForcePivot(),
                CommandFactory.IntakeBallsON(),
                        new WaitCommand(2.2),
                        CommandFactory.IntakeBallsOFF()).alongWith(new FollowChoreoTrajectory(traj)),
                        CommandFactory.AutoAimShoot().raceWith(new WaitCommand(6)));
    }
                    
    public static Command trench_left_left_mid_chill() {
        // Timer timer = new Timer();
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_left_left_mid_chill");
        return new SequentialCommandGroup(

                new SequentialCommandGroup(
                    new WaitCommand(0.8),
                    // Commands.runOnce(timer::restart),
                    // new WaitUntilCommand(() -> timer.hasElapsed(0.8)),
                    new ForcePivot(), //1.3
                    CommandFactory.IntakeBallsON(),
                    new WaitCommand(2.6), //
                    // new WaitUntilCommand(() -> timer.hasElapsed(4.4)),
                    CommandFactory.IntakeBallsOFF()
                    ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.AutoAimShoot(),
                new JiggleBallsDrivetrainNoRolla()
            );
    }

    /*
     * Enum for the different autos. Contains a name and a mechCommands array. The
     * mechCommands array contains
     * all the commands that the mechanisms will use (stuff that is unrelated to the
     * drivetrain). These commands
     * will be executed in the order they are in the array during the auto path.
     * Refer to runAutoCommand(AutoType auto).
     */
    public enum AutoPath {
        // when writing enums, if you want multiple mechCommands to run before the next
        // path, put them in a sequential command group
        // if you want those mechCommands to run in parallel, put them in a
        // parallelCommandGroup
        // if you want to run a mechCommand or mechCommandGroup in parallel with a path,
        // create a boolean array with true values corresponding to the mechCommands you
        // want to run in parallel.

        depo_simple("depo_simple", depo_simple()),
        mid("mid", mid()),
        mid_to_depo("mid_to_depo", mid_to_depo()),
        mid_tuning("mid_tuning", mid_tuning()),
        mid_right("mid_right", mid_right()),
        trench_left_left_mid_chill("trench_left_left_mid_chill", trench_left_left_mid_chill()),
        trench_right_right_mid_chill("trench_right_right_mid_chill", trench_right_right_mid_chill());

        String name;
        Command autoCommand;

        private AutoPath(String a, Command autoCommand) {
            name = a;
            this.autoCommand = autoCommand;
        }
    }

}
