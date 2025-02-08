// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {   

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1;    // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = 0.10;
        // gear ratio from thrifty swerve https://thethriftybot.com/products/thrify-swerve gear ratio options (pinion size 12 + second stage gear 16t? (only confirmed pinion))
        public static final double kDriveMotorGearRatio = 1/6.0;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;//(12*14)/(72*50) based on #of teeth
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 16;
        public static final int kFrontLeftTurningMotorPort = 15;

        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kFrontRightTurningMotorPort = 21;

        public static final int kBackRightDriveMotorPort = 26;
        public static final int kBackRightTurningMotorPort = 25;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // OLD OFFSETS
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRot = 0.849;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRot = 0.597;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRot = 0.148;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRot = 0.613;

        // OFFSETS
        // If robot is positively off, subtract
        // Else, add
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 61.645715;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 212.642091;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 142.585028;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 189.405267;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.2;
        public static final double kSlowButtonTurnModifier = 0.5;
        public static final double kPDrift = 0.1;
    }

    public static final class IntakeConstants {
        public static final int frontWheelID = 42;
        public static final int leftWheelID = 41;
        public static final int rightWheelID = 40;
    }

    public static final class LEDConstants {
        public static final int numLEDsPerStrip = 36;
        public static final int numLEDsPerColor = 3;
    }

    public static final class GameConstants {
        public static final int Robot = 0;
        public static final int Auto = 1;
        public static final int TeleOp = 2;
    }

    // public static final class ArmConstants {
    //     public static final int leftMotorID = 43;
    //     public static final int rightMotorID = 44;
    //     public static final double kP =  0.1;
    //     public static final double kD =  0.01;
    //     public static final double speakerEncoder = -34; //-30;
    //     public static final double farSpeakerEncoder = -52; // 50->45, might need to adjust TO-DO
    //     public static final double ampEncoder = -150;
    //     public static final double armStartingPos = -120; 
    //     public static final double armHover = -2;
    // }

    public static final class PoseEstimatorConstants {
        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 208
        public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

        // See
        // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
        // page 197
        public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

        // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
        // pages 4 and 5
        public static final double kFarTgtXPos = Units.feetToMeters(54);
        public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
        public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

        public static final Pose3d kFarTargetPose =
            new Pose3d(
                new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(180))
        );

        public static final Transform3d kCameraToRobot = 
            new Transform3d(
                new Translation3d(-0.33,-0.17,0.36), 
                new Rotation3d(0,-45,180)
        );
    }

    // public static final class ClimbConstants {
    //     public static final int leftMotorID = 50;
    //     public static final int rightMotorID = 51;
    //     public static final double kP =  1 / 1;
    // }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverRB = 6;

        public static final double kDeadband = 0.05;

        //BUTTONS

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int L3 = 9;
        public static final int R3 = 10;

        //AXES

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;

        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }

    public static final class LimelightConstants {
        public static final String tagName = "limelight-tags";
        public static final String gamePieceName = "limelight-gp";
        public static final int Estimate_Distance = 20;
    }

    public static final class Colors {
        public static final Color red = Color.kRed;
        public static final Color blue = Color.kBlue;
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = Color.kWhite;
        public static final Color uRed = new Color(20, 0 , 0);
        public static final Color uDarkOrange = new Color(254,17,1);
        public static final Color uGreen = new Color(0, 7, 0);
        public static final Color uOrange = new Color(255, 25, 0);
        public static final Color[] uColors = {uRed, uDarkOrange, uGreen, uOrange};
    }

    public static final class AprilTagPositions {
        // all in m, 0 is left
        public static final Pose2d redPickupLeft1 = new Pose2d(16.697198, 0.65532, new Rotation2d(126));
        public static final Pose2d redPickupRight2 = new Pose2d(16.697198, 7.3964799999999995, new Rotation2d(234));
        public static final Pose2d redProcessor3 = new Pose2d(11.560809999999998, 8.05561, new Rotation2d(270));
        public static final Pose2d redReefLeft6 = new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(30));
        public static final Pose2d redReefCenter7 = new Pose2d(13.890498, 4.0259, new Rotation2d(0));
        public static final Pose2d redReefRight8 = new Pose2d(13.474446, 4.745482, new Rotation2d(30));
        public static final Pose2d redReefBackRight9 = new Pose2d(12.643358, 4.745482, new Rotation2d(120));
        public static final Pose2d redReefBackCenter10 = new Pose2d(12.227305999999999, 4.0259, new Rotation2d(180));
        public static final Pose2d redReefBackLLeft11 = new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(240));
        public static final Pose2d bluePickupRight12 = new Pose2d(0.851154, 0.65532, new Rotation2d(54));
        public static final Pose2d bluePickupLeft13 = new Pose2d(0.851154, 7.3964799999999995, new Rotation2d(306));
        public static final Pose2d blueProcessor16 = new Pose2d(5.9875419999999995, -0.0038099999999999996, new Rotation2d(90));
        public static final Pose2d blueReefRight17 = new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(240));
        public static final Pose2d blueReefCenter18 = new Pose2d(3.6576, 4.0259, new Rotation2d(180));
        public static final Pose2d blueReefLeft19 = new Pose2d(4.073905999999999, 4.745482, new Rotation2d(120));
        public static final Pose2d blueReefBackLeft20 = new Pose2d(4.904739999999999, 4.745482, new Rotation2d(60));
        public static final Pose2d blueReefBackCenter21 = new Pose2d(5.321046, 4.0259, new Rotation2d(0));
        public static final Pose2d blueReefBackRight22 = new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(300));
    }

    public static final class Measurements {
        // all in m
        public static final double robotCenterToFront = 0.7;
        public static final double robotSideOffset = 0;
    }

    // public static final double degreesToRadians = 

    public static final class RobotPositions {
        public static final Pose2d redCenter = new Pose2d(11.62, 4.0259, new Rotation2d(0));
        public static final Pose2d redCenterSafe = new Pose2d(11.35, 4.0259, new Rotation2d(0));
        //y_{17}.x-r_{l}\cdot\cos\left(60\right)+r_{w}\sin\left(60\right),
        //y_{17}.y-r_{l}\cdot\sin\left(60\right)-r_{w}\cdot\cos\left(60\right)
        public static final Pose2d redPickupLeft1 = new Pose2d(AprilTagPositions.redPickupLeft1.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), 
        AprilTagPositions.redPickupLeft1.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54)) - Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(306));

        public static final Pose2d redPickupRight2 = new Pose2d(AprilTagPositions.redPickupRight2.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54)) + Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), 
        AprilTagPositions.redPickupRight2.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(54));

        public static final Pose2d redProcessor3 = new Pose2d(AprilTagPositions.redProcessor3.getX()+Measurements.robotSideOffset, AprilTagPositions.redProcessor3.getY()-Measurements.robotCenterToFront, new Rotation2d(90));

        public static final Pose2d redReefLeft6 = new Pose2d(AprilTagPositions.redReefLeft6.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.redReefLeft6.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(150));

        public static final Pose2d redReefCenter7 = new Pose2d(AprilTagPositions.redReefCenter7.getX()+Measurements.robotCenterToFront, AprilTagPositions.redReefCenter7.getY()+Measurements.robotSideOffset, new Rotation2d(180));

        public static final Pose2d blueReefRight17 = new Pose2d(AprilTagPositions.blueReefRight17.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.blueReefRight17.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.cos(Units.degreesToRadians(60)), new Rotation2d(210));

        // public static final Pose2d blueCenter = new Pose2d(AprilTagPositions.blueReefBackCenter21.getX()-
    }
}
