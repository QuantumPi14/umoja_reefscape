package frc.robot.subsystems;

import com.studica.frc.AHRS;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.PoseEstimatorConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegree,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // private AprilTagFieldLayout fieldAprilTags= AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    //     DriveConstants.kDriveKinematics, new Rotation2d(),
    //     new SwerveModulePosition[] {
    //       frontLeft.getPosition(),
    //       frontRight.getPosition(),
    //       backLeft.getPosition(),
    //       backRight.getPosition()
    //     }, new Pose2d());
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d()
    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);

                zeroHeading();
                resetEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        // Configure AutoBuilder last
        // AutoBuilder.configureHolonomic(
        //         this::getPose, // Robot pose supplier
        //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         this::setModuleStatesFromSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //                 new PIDConstants(1, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(Constants.ModuleConstants.kPTurning, 0.0, 0.0), // Rotation PID constants
        //                 Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, // Max module speed, in m/s
        //                 Constants.DriveConstants.kRobotRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
        //         ),
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    
    // public void resetAbsoluteEncoders(){
    //     frontLeft.resetAbsoluteEncoders();
    //     frontRight.resetAbsoluteEncoders();
    //     backLeft.resetAbsoluteEncoders();
    //     backRight.resetAbsoluteEncoders();
    // }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(-gyro.getYaw(), 360);
        // double heading = (-gyro.getYaw() + 360)%180;
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        SmartDashboard.putNumber("HEADING", heading);
        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // public void resetOdometry(Pose2d pose) {
    //     System.out.println("ODOMETRY RESET");
    //     poseEstimator.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), new SwerveModulePosition[] {
    //         frontLeft.getPosition(),
    //         frontRight.getPosition(),
    //         backLeft.getPosition(),
    //         backRight.getPosition()
    //     }, pose);
    // }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("FL", frontLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("FR", frontRight.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BL", backLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BR", backRight.getAbsoluteEncoderDegree());

        SmartDashboard.putNumber("T FL", Math.toDegrees(frontLeft.getTurningPosition())%360);
        SmartDashboard.putNumber("T FR", Math.toDegrees(frontRight.getTurningPosition())%360);
        SmartDashboard.putNumber("T BL", Math.toDegrees(backLeft.getTurningPosition())%360);
        SmartDashboard.putNumber("T BR", Math.toDegrees(backRight.getTurningPosition())%360);
        SmartDashboard.putNumber("YAW", gyro.getYaw());

        // if(RobotContainer.gameState!=GameConstants.TeleOp){
        //     poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw()),
        //     // odometer.update(getRotation2d(),
        //         new SwerveModulePosition[] {
        //         frontLeft.getPosition(),
        //         frontRight.getPosition(),
        //         backLeft.getPosition(),
        //         backRight.getPosition()
        //     });

        //     // var result = RobotContainer.camera.getLatestResult();
        //     // boolean hasTargets = result.hasTargets();
        //     // SmartDashboard.putBoolean("Has Targets", hasTargets);
        //     // if(hasTargets){
        //     //     var imageCaptureTime = result.getTimestampSeconds();
        //     //     var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
        //     //     int id = result.getBestTarget().getFiducialId();
        //     //     var tagFromId = fieldAprilTags.getTagPose(id);
        //     //     if (tagFromId != null) {
        //     //         Pose3d tag = tagFromId.get();
        //     //         Pose3d poseTarget = new Pose3d(tag.getTranslation(), tag.getRotation());
        //     //         SmartDashboard.putNumber("X", tag.getX());
        //     //         SmartDashboard.putNumber("Y", tag.getY());
        //     //         SmartDashboard.putNumber("Z", tag.getZ());
        //     //         var camPose = poseTarget.transformBy(camToTargetTrans.inverse());
        //     //         poseEstimator.addVisionMeasurement(camPose.transformBy(PoseEstimatorConstants.kCameraToRobot).toPose2d(), imageCaptureTime);
        //     //     }
        //     // }
        // }
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }
}
