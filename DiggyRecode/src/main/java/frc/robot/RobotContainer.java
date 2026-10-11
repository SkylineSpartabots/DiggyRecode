// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
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
//import frc.robot.Commands.Automation.JiggleBallsDrivetrain;
import frc.robot.Commands.Convayor.SetConveyor;
import frc.robot.Commands.Indexer.SetIndexer;
import frc.robot.Commands.Intake.SetIntake;
import frc.robot.Commands.Pivot.ForcePivot;
import frc.robot.Commands.Shooter.SetShooter;
// import frc.robot.Subsystems.Climb;
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
    private Indexer   indexer =                                  Indexer.getInstance();
    private Intake    intake =                                    Intake.getInstance();
    private Conveyor  conveyor =                                Conveyor.getInstance();
    private Shooter   shooter =                                  Shooter.getInstance();
    private Pivot     pivot =                                      Pivot.getInstance();
    private Quest     quest =                         Quest.getInstance();
    // private Climb climb = Climb.getInstance();

    // --- Drive control and controllers ---
    private DriveControlSystems control = DriveControlSystems.getInstance();
    /** Driver controller (port 0) — controls drivetrain and shooting. */
    public final CommandXboxController driver = new CommandXboxController(0);
    /** Operator controller (port 1) — reserved for secondary mechanisms. */
    public final CommandXboxController opp = new CommandXboxController(1);

    /**
     * Intake pivot position for the left-bumper toggle.
     * Starts raised. Clockwise (positive voltage) pulls the pivot back up;
     * counterclockwise (negative voltage) deploys it.
     */
    private boolean pivotLowered = false;

    public RobotContainer() { configureBindings(); }

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

        //opp.povUP().onTrue(new InstantCommand(()->));
        // Left bumper toggles the intake pivot. Raised -> -4 V to deploy, lowered -> +2 V to stow.
        driver.leftBumper().onTrue(togglePivot());
        // Right bumper: distance RPS from odometry, and face the hub. Left stick still drives.
        // Right trigger gives the right stick rotation back. The flywheel keeps spinning.
        driver.rightBumper().onTrue(new AlignToGoal(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()));
        driver.rightBumper().onTrue(CommandFactory.DriverShoot());

        driver.leftTrigger().onTrue(Commands.runOnce(control::turnOnAutoAim));
        driver.rightTrigger().onTrue(Commands.runOnce(control::turnOffAutoAim));

        // driver.povDown().onTrue(new JiggleBallsDrivetrain(driver));
        // Fixed 15 rps test. Shooter only — feeding is POV up.
        driver.povDown().onTrue(new SetShooter(15));

        // Field-relative fix: point the robot away from your driver station, press Start.
        // Alliance-aware, keeps XY. Works while disabled so it can be done before the match.
        driver.start().onTrue(Commands.runOnce(drivetrain::resetHeading).ignoringDisable(true));

        driver.a().onTrue(CommandFactory.IntakeBallsON());
       

        //reset odo
        driver.x().onTrue(Commands.runOnce(() -> quest.anchorQuest(new Pose3d(drivetrain.getState().Pose))).ignoringDisable(true));
        //driver.x().onTrue(new ForcePivot(5));

        driver.b().onTrue(CommandFactory.IntakeBallsOFF());
        // POV up feeds whatever speed the shooter is already at. The match shot is the right bumper.
        driver.povUp().onTrue(CommandFactory.Feed());

        driver.povLeft().onTrue(repositionAt(resetPose.TRENCH_LEFT));
        //State flipper 
        driver.povRight().onTrue(CommandFactory.ReverseFeed());
        opp.povDown().onTrue(CommandFactory.AllOff().alongWith(Commands.runOnce(control::turnOffAutoAim)));

        // Operator: place the robot on a known spot and press to re-seed position.
        // Left/right are from your own driver station. Heading is kept; Start fixes heading.
        opp.povLeft().onTrue(repositionAt(resetPose.TRENCH_LEFT));
        opp.povRight().onTrue(repositionAt(resetPose.TRENCH_RIGHT));
        opp.povUp().onTrue(repositionAt(resetPose.MIDDLE));
        opp.start().onTrue(Commands.runOnce(drivetrain::resetHeading).ignoringDisable(true));

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
     * Toggles the intake pivot between raised and lowered.
     * Lowering applies -4 V (counterclockwise deploy). Raising applies +2 V (clockwise stow).
     * {@link #pivotLowered} updates only after the move finishes.
     */
    //toggles the pivot
    private Command togglePivot() {
        return Commands.either(
                new ForcePivot(2.0).andThen(Commands.runOnce(() -> pivotLowered = false)),
                new ForcePivot(-4.0).andThen(Commands.runOnce(() -> pivotLowered = true)),
                () -> pivotLowered);
    }

    /** Re-seeds odometry at a known spot. Runs while disabled for pre-match setup. */
    private Command repositionAt(resetPose location) {
        return Commands.runOnce(() -> drivetrain.resetOdoDynamic(location)).ignoringDisable(true);
    }

    /**
     * Turns off all mechanisms in parallel — intake, conveyor, indexer, shooter.
     */
    public Command allOff() {
        return new ParallelCommandGroup(
                intake.setState(IntakeStates.OFF),
                conveyor.setState(ConveyorStates.OFF),
                indexer.setState(IndexerStates.OFF),
                new InstantCommand(() -> shooter.setVelocity(0), shooter));
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
