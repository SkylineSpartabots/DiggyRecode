// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.io.SequenceInputStream;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Commands.CommandFactory;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.SetPivot;
import frc.robot.Commands.Shooter.RampShooterWithDistance;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;

public final class Autos {
    static Timer path_time = new Timer();

    public static Command getAutoCommand(AutoPath autoPath) {
        return autoPath.autoCommand;
    }
    //hi your name is siggy! ok so now i will code
    // go in a direction if i ask you to using the controller
    //after that, make sure to score!
    //i believe in you to make every shot!
    //then, go to the middle and shuttle balls to our side
    //after that, try your best to score more balls
    //the, go climb during endgame! i believe in you and your skill <3
    //remember not to break! we will be proud of you no matter what
    //-sasha



    // public static Command mid_right() {
    //     Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid_right");
    //     return new SequentialCommandGroup(
    //             new WaitCommand(2.25),
    //             new SetPivot(PivotStates.DEPLOY),
    //             CommandFactory.ShootAtDistance()
    //     ).alongWith(new FollowChoreoTrajectory(traj));
    // }

    // used autos -----------------------------------


    public static Command mid_evil_depo() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid_evil_depo");
        return new SequentialCommandGroup(
            new SequentialCommandGroup( 
                new WaitCommand(1.8),
                new SetPivot(PivotStates.DEPLOY),
                new WaitCommand(1),
                new SetPivot(PivotStates.OFF),
                new SetIntake(IntakeStates.ON),
                new WaitCommand(3)
                ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.ShootAtDistance()
                );
    }

    public static Command trench_left_mid() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_left_mid");
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new SetPivot(PivotStates.DEPLOY),   
                    new WaitCommand(1),
                    CommandFactory.IntakeBallsON(),
                    new WaitCommand(0.6),
                    new SetPivot(PivotStates.OFF),
                    new WaitCommand(1.7), // 3
                    CommandFactory.IntakeBallsOFF()
                    ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.ShootAtDistance(),
                new SetPivot(PivotStates.RETRACT)
            );
    }

    public static Command trench_left_mid_freakazoid() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_left_mid_freakazoid");
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new SetPivot(PivotStates.DEPLOY),   
                    new WaitCommand(1),
                    CommandFactory.IntakeBallsON(),
                    new WaitCommand(0.6),
                    new SetPivot(PivotStates.OFF),
                    new WaitCommand(1.7), // 3
                    CommandFactory.IntakeBallsOFF()
                    ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.ShootAtDistance(),
                new SetPivot(PivotStates.RETRACT)
            );
    }

    public static Command trench_right_mid() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_right_mid");
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new SetPivot(PivotStates.DEPLOY),   
                    new WaitCommand(0.9),
                    CommandFactory.IntakeBallsON(),
                    new WaitCommand(0.1),
                    new SetPivot(PivotStates.OFF),
                    new WaitCommand(1.7), // 3
                    CommandFactory.IntakeBallsOFF()
                    ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.ShootAtDistance(),
                new SetPivot(PivotStates.RETRACT)
            );
    }

    public static Command trench_left_evil() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("trench_left_evil");
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new SetPivot(PivotStates.DEPLOY),   
                    new WaitCommand(1),
                    CommandFactory.IntakeBallsON(),
                    new WaitCommand(0.25),
                    new SetPivot(PivotStates.OFF),
                    new WaitCommand(2.05), // 3.3
                    CommandFactory.IntakeBallsOFF()
                    ).alongWith(new FollowChoreoTrajectory(traj)),

                CommandFactory.ShootAtDistance(),
                new SetPivot(PivotStates.RETRACT)
            );
    }

    public static Command mid() {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory("mid");
        return new SequentialCommandGroup(
                new WaitCommand(2),
                new SetPivot(PivotStates.DEPLOY),
                new WaitCommand(1),
                new SetPivot(PivotStates.OFF),
                CommandFactory.ShootAtDistance()
        ).alongWith(new FollowChoreoTrajectory(traj));
    }

    public enum AutoPath {
        trench_left_mid("trench_left_mid", trench_left_mid()),
        trench_left_mid_freakazoid("trench_left_mid_freakazoid", trench_left_mid_freakazoid()),
        trench_right_mid("trench_right_mid", trench_right_mid()),
        trench_left_evil("trench_left_evil", trench_left_evil()),
        mid("mid", mid()),
        mid_evil_depo("mid_evil_depo", mid_evil_depo());

        String name;
        Command autoCommand;

        private AutoPath(String a, Command autoCommand) {
            name = a;
            this.autoCommand = autoCommand;
        }
    }

}
