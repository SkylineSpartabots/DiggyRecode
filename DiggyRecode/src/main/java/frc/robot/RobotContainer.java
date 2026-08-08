// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.DriveControlSystems;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain.resetPose;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;
import frc.robot.Subsystems.Vision.Quest;
import frc.robot.Commands.CommandFactory;
import frc.robot.Commands.Automation.AlignToGoal;
import frc.robot.Commands.Automation.JiggleBallsDrivetrain;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.ForcePivot;
import frc.robot.Commands.Shooter.SetShooter;
// import frc.robot.Subsystems.Climb; // Climb is preserved but not in use this season
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Conveyor.ConveyorStates;

/**
 * RobotContainer wires all subsystems and binds driver/operator controls.
 * All button mappings for teleop are configured in
 * {@link #configureBindings()}.
 *
 * NOTE: Climb subsystem code is preserved but not instantiated or bound this
 * season.
 */
public class RobotContainer {

    // --- Active subsystem singletons ---
    private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private Indexer indexer = Indexer.getInstance();
    private Intake intake = Intake.getInstance();
    private Conveyor conveyor = Conveyor.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Pivot pivot = Pivot.getInstance();
    // private Climb climb = Climb.getInstance();

    // --- Drive control and controllers ---
    private DriveControlSystems control = DriveControlSystems.getInstance();
    /** Driver controller (port 0) — controls drivetrain and shooting. */
    public final CommandXboxController driver = new CommandXboxController(0);
    /** Operator controller (port 1) — reserved for secondary mechanisms. */
    public final CommandXboxController opp = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configures all driver and operator button bindings.
     * Final competition bindings are in the labeled section;
     * testing/sysid bindings are commented out below.
     */
    private void configureBindings() {
        /* DT bindings */

        // Default drivetrain command: field-centric drive from left stick (translate) +
        // right stick (rotate)
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> control.drive(
                                -driver.getLeftY(),
                                -driver.getLeftX(),
                                -driver.getRightX())));

        // final bindings -----------------------------------------------

        driver.leftBumper().onTrue(CommandFactory.IntakeBallsON());// FIX COMMAND // top buttons
        driver.rightBumper().onTrue(CommandFactory.IntakeBallsOFF());

        driver.leftTrigger().onTrue(new InstantCommand(() -> control.turnOnAutoAim())); // bottom buttons
        driver.rightTrigger().onTrue(new InstantCommand(() -> control.turnOffAutoAim()));

        // driver.povDown().onTrue(new JiggleBallsDrivetrain(driver));
        driver.povDown().onTrue(CommandFactory.LobAtRps(15));

        driver.start().onTrue(new InstantCommand(() -> drivetrain.resetOdo()));

        driver.a().onTrue(CommandFactory.AllOff());
        driver.y().onTrue(new ForcePivot());
        driver.x().onTrue(new ForcePivot(5));

        driver.b().onTrue(CommandFactory.ShootAtDistance());

        driver.povLeft().onTrue(new InstantCommand(() -> drivetrain.resetOdoDynamic(resetPose.TRENCH_LEFT)));
        driver.povRight().onTrue(CommandFactory.ReverseFeed());

        // driver.povLeft().onTrue(new InstantCommand(() ->
        // drivetrain.resetOdoDynamic(resetPose.TRENCH_LEFT)));
        // driver.povRight().onTrue(new InstantCommand(() ->
        // drivetrain.resetOdoDynamic(resetPose.TRENCH_RIGHT)));
        // driver.povUp().onTrue(new InstantCommand(() ->
        // drivetrain.resetOdoDynamic(resetPose.MIDDLE)));

        // driver.povRight().onTrue(CommandFactory.LobAtMeter(4));

        // testing bindings -----------------------------------------------

        // driver.b().onTrue(new InstantCommand(() -> intake.setVelocity(35))); //
        // intake
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

    /**
     * Turns off all mechanisms in parallel — intake, conveyor, indexer, shooter.
     */
    public Command allOff() {
        return new ParallelCommandGroup(
                intake.setState(IntakeStates.OFF),
                conveyor.setState(ConveyorStates.OFF),
                indexer.setState(IndexerStates.OFF),
                new InstantCommand(() -> shooter.setVoltage(0)));
    }

    /**
     * Runs conveyor + indexer + shooter at a fixed 15 rps — used for testing ball
     * cycling.
     */
    public Command chud() {
        return new ParallelCommandGroup(
                conveyor.setState(ConveyorStates.ON),
                indexer.setState(IndexerStates.ON),
                new InstantCommand(() -> shooter.setVelocity(15)));
    }

    /** Runs conveyor and intake together — used to test ball feeding. */
    public Command chud2() {
        return new ParallelCommandGroup(
                conveyor.setState(ConveyorStates.ON),
                intake.setState(IntakeStates.ON));
    }

}
