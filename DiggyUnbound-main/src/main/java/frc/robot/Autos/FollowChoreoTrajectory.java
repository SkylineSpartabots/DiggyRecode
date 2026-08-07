// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.util.Optional;
import java.util.Queue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.DriveControlSystems;
import frc.robot.Subsystems.Vision.Quest;
import gg.questnav.questnav.QuestNav;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowChoreoTrajectory extends Command {
  private Optional<Trajectory<SwerveSample>> trajectoryOpt;
  private Trajectory<SwerveSample> trajectory = null;

  private CommandSwerveDrivetrain s_Swerve;
  private Quest quest;

  private Optional<Pose2d> startPose;
  private Timer timer;
  
  private PIDController xController = new PIDController(0.67, 0, 0);
  private PIDController yController = new PIDController(0.67, 0, 0);

  private static final PIDController thetaController = new PIDController(2.7, 0, 0.02); //tuned. -ethan


  public FollowChoreoTrajectory(Optional<Trajectory<SwerveSample>> traj) {
    this.quest = Quest.getInstance();

    s_Swerve = CommandSwerveDrivetrain.getInstance();
    timer = new Timer();

    trajectoryOpt = traj;
    
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    if (trajectoryOpt.isPresent()) {
      trajectory = trajectoryOpt.get();

      if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        trajectory = trajectory.flipped(); //flip for red
      }

      System.out.println(" trajectory optinales present");
    }

    System.out.println("AAAAAAAAAAAAAA");

    if (trajectory != null){
      startPose = trajectory.getInitialPose(false); 

      System.out.println("lala" + startPose.toString());

      s_Swerve.resetOdo(startPose.get());
      // quest.anchorQuest(new Pose3d(startPose.get()));
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    if(trajectory != null){
      Optional<SwerveSample> sample = trajectory.sampleAt(timer.get(), false);
      FollowChoreoSample(sample.get());
    }
  }

  @Override
  public void end(boolean interrupted) {
      timer.stop();
      s_Swerve.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    // System.out.println(trajectory);
    return trajectory != null ? timer.hasElapsed(trajectory.getTotalTime()+ 0.2) : true;
  }

    public void FollowChoreoSample(SwerveSample sample) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // System.out.println(sample.x + " " + sample.y + " " + sample.heading);

        // Get the current pose of the robot
        Pose2d pose = s_Swerve.getState().Pose;

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading)
            );

        // System.out.println(speeds.toString());

Alliance alliance = DriverStation.getAlliance().get();
ForwardPerspectiveValue perspective = alliance == Alliance.Red 
    ? ForwardPerspectiveValue.OperatorPerspective  // or red-specific
    : ForwardPerspectiveValue.BlueAlliance;

        s_Swerve.setControl(
            new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
            .withForwardPerspective(perspective)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
        );

    }

}