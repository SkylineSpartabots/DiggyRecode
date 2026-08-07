// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.tools.DocumentationTool.Location;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.DriveControlSystems;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;
import frc.robot.Subsystems.Vision.Quest;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.Commands.CommandFactory;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.SetPivot;
import frc.robot.Commands.Pivot.SetPivotTimed;
import frc.robot.Commands.Shooter.SetShooter;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Conveyor.ConveyorStates;

public class RobotContainer {

    private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private Indexer indexer = Indexer.getInstance();
    private Intake intake = Intake.getInstance();

    private Conveyor conveyor = Conveyor.getInstance();
    private Shooter shooter = Shooter.getInstance();

    private Pivot pivot = Pivot.getInstance();
    private Quest quest = Quest.getInstance();

    private DriveControlSystems control = DriveControlSystems.getInstance();
    public final CommandXboxController driver = new CommandXboxController(0);
    // public final CommandXboxController opp = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        /* DT bindings */

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> control.drive(
                    -driver.getLeftY(), 
                    -driver.getLeftX(), 
                    -driver.getRightX()
                )
            ) // Drive counterclockwise with negative X (left)
        );
        
        // final bindings -----------------------------------------------

        driver.leftBumper().onTrue(CommandFactory.IntakeBallsON());// FIX COMMAND // top buttons
        driver.rightBumper().onTrue(CommandFactory.IntakeBallsOFF());

        driver.leftTrigger().onTrue(new InstantCommand(() -> control.turnOnAutoAim())); //bottom buttons
        driver.rightTrigger().onTrue(new InstantCommand(() -> control.turnOffAutoAim()));

        // blue
        driver.povDown().onTrue(new InstantCommand(() -> quest.anchorQuest(new Pose3d(Constants.ResetPoses.blue_Mid))));
        //red
        driver.povUp().onTrue(new InstantCommand(() -> quest.anchorQuest(new Pose3d(Constants.ResetPoses.red_Mid))));
        
        driver.povRight().onTrue(new SetPivotTimed(PivotStates.DEPLOY));
        driver.povLeft().onTrue(new SetPivotTimed(PivotStates.RETRACT));

        driver.start().onTrue(new InstantCommand(() -> drivetrain.resetOdo()));

        driver.a().onTrue(CommandFactory.AllOff());

        driver.b().onTrue(CommandFactory.ShootAtDistance());
        
        driver.y().onTrue(new SetPivotTimed(PivotStates.HARD_DEPLOY));
        // driver.y().onTrue(CommandFactory.LobAtRps(45));
        // driver.y().onTrue(CommandFactory.chud());

        driver.x().onTrue(CommandFactory.LobAtMeter(2.5));
        // driver.x().onTrue(CommandFactory.LobAtRps(40));

        driver.back().onTrue(CommandFactory.reverseStates());
        
        
        // driver.povLeft().onTrue(new InstantCommand(() -> drivetrain.resetOdoDynamic(resetPose.TRENCH_LEFT)));
        // driver.povRight().onTrue(new InstantCommand(() -> drivetrain.resetOdoDynamic(resetPose.TRENCH_RIGHT)));
        // driver.povUp().onTrue(new InstantCommand(() -> drivetrain.resetOdoDynamic(resetPose.MIDDLE)));


        // driver.povRight().onTrue(CommandFactory.LobAtMeter(4));



        // testing bindings -----------------------------------------------
        
        // driver.b().onTrue(new InstantCommand(() -> intake.setVelocity(35))); // intake 
        // driver.a().onTrue(new InstantCommand(() -> intake.setVoltage(0))); // intake
        
        // driver.a().onTrue(allOff()); // intake

        // driver.x().onTrue(CommandFactory.IntakeBallsON()); // intake 
        // driver.a().onTrue(CommandFactory.IntakeBallsOFF()); // intake 
        // driver.b().onTrue(new SetIntake(IntakeStates.OFF)); // intake 

        // driver.a().onTrue(new InstantCommand(() -> drivetrain.resetOdo())); // intake 

        // driver.povLeft().onTrue(new SetConveyor(ConveyorStates.ON)); // intake 
        // driver.povRight().onTrue(new SetConveyor(ConveyorStates.OFF)); // intake 

        // driver.povUp().onTrue(CommandFactory.LobAtRps(25)); // intake 
        // driver.povLeft().onTrue(CommandFactory.LobAtRps(50)); // intake 
        // driver.povRight().onTrue(CommandFactory.LobAtRps(75)); // intake 
        // driver.povDown().onTrue(CommandFactory.LobAtRps(90)); // intake 

        // driver.x().onTrue(chud2());
        // driver.povUp().onTrue(new InstantCommand(() -> control.turnOnAutoAim()));
        // driver.povDown().onTrue(new InstantCommand(() -> control.turnOffAutoAim()));

    
        // driver.x().onTrue(new InstantCommand(() -> shooter.setVelocity(75)));


        // driver.y().onTrue(new InstantCommand(() -> conveyor.setVoltage(7)));

        /* Sysid Bindings IGNORE TS */

        // driver.povLeft().onTrue(new InstantCommand(() -> SignalLogger.start()));
        // driver.povRight().onTrue(new InstantCommand(() -> SignalLogger.stop()));

        // driver.x().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // driver.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        // driver.y().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // driver.a().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
    }
}
