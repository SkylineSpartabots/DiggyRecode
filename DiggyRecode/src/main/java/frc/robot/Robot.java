// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.LimeLight;
import frc.robot.Subsystems.Vision.Quest;
import frc.robot.Autos.Autos;
import frc.robot.Autos.Autos.AutoPath;
// import frc.robot.Subsystems.Climb; 
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private CommandSwerveDrivetrain drivetrain;
    private LimeLight limeLight;
    private Quest quest;

    SendableChooser<Autos.AutoPath> autoChooser = new SendableChooser<Autos.AutoPath>();

    public Robot() {
        // Initialize all subsystem singletons so they configure their hardware at
        // startup
        drivetrain = CommandSwerveDrivetrain.getInstance();
        limeLight = LimeLight.getInstance();
        quest = Quest.getInstance();
        Indexer.getInstance();
        Intake.getInstance();
        Conveyor.getInstance();
        Shooter.getInstance();
        Pivot.getInstance();
        // Climb.getInstance();

        // Register auto options in the dashboard chooser
        autoChooser.setDefaultOption("mid", AutoPath.mid);
        autoChooser.addOption("mid_to_depo", AutoPath.mid_to_depo);
        autoChooser.addOption("trench_left_left_mid_chill", AutoPath.trench_left_left_mid_chill);
        autoChooser.addOption("mid_right", AutoPath.mid_right);

        SmartDashboard.putData("Auto choices", autoChooser);

        // SignalLogger.setPath("/media/sdb1/ctre-logs/");

        m_robotContainer = new RobotContainer();
    }

    /**
     * Runs the command scheduler every robot loop (~20ms). Must never be blocked.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    /** While disabled, keep limelight pipeline updated for vision pre-loading. */
    @Override
    public void disabledPeriodic() {
        limeLight.updateLimelight();
        // quest.anchorQuest();
    }

    @Override
    public void disabledExit() {
    }

    /** Fetch and schedule the auto routine selected on the dashboard. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = Autos.getAutoCommand(autoChooser.getSelected());

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /** Cancel any running auto command when teleop starts. */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    /** Cancel all commands when entering test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
