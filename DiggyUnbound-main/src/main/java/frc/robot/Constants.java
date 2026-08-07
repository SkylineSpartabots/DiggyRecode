package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public static final class ResetPoses {
        public static final Pose2d blue_TrenchLeft = new Pose2d(4.4, 7.17, new Rotation2d());
        public static final Pose2d blue_TrenchRight = new Pose2d(4.4, 0.89, new Rotation2d());
        public static final Pose2d blue_Mid = new Pose2d(3.6, 4.04, new Rotation2d());

        public static final Pose2d red_TrenchLeft = new Pose2d(12.13, 0.89, new Rotation2d());
        public static final Pose2d red_TrenchRight = new Pose2d(12.13, 7.17, new Rotation2d());
        public static final Pose2d red_Mid = new Pose2d(12.95, 4.04, Rotation2d.fromDegrees(180));
    }

    public static final class HardwarePorts {
        public static final int intake = 20;
        public static final int pivot = 21;

        public static final int conveyor = 30;

        public static final int indexer= 40;

        public static final int shooterTL = 50;
        public static final int shooterBL = 51;
        public static final int shooterTR = 52;
        public static final int shooterBR = 53;

        public static final int climbL = 60;
        public static final int climbR = 61; 
    }

    public static final class CurrentLimits {
        public static final int shooterSupply = 50;
        public static final int shooterStator = 95;

        public static final int pivotSupply = 100;
        public static final int pivotStator = 140;

        // public static final int intakeSupply = 0;
        public static final int intakeStator = 100;

        // public static final int indexerSupply = 0;
        public static final int indexerStator = 85;

        public static final int conveyorSupply = 45;
        public static final int conveyorStator = 65;
     }
    
    public static final double stickDeadband = 0.1;
    public static final double triggerDeadzone = 0.2;

    public static double MaxSpeed = 5.5; //can be lowered during testing
    public static double MaxAcceleration = 1.2542976; //can be lowered during testing
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static double MaxAngularVelocity = 2 * Math.PI;

    public static double shooterRadiusM = 0.0508;
    public static double shooterAngleRad = 1.1694;
    public static double shooterHeightM = 0.28989;
    
    public static final class FieldConstants {
        public static Translation3d blueGoal = new Translation3d(4.6256,4.0345, 1.8288); // meters
        public static Translation3d redGoal = new Translation3d(11.9079, 4.0345, 1.8288); // meters, double check coords at some point
    }
}
