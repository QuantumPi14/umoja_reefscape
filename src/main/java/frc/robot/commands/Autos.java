package frc.robot.commands;

import java.io.ObjectInputFilter.Config;
import java.util.List;
import java.util.Set;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;


public class Autos {
    private final SwerveSubsystem swerveSubsystem;
    private final TrajectoryConfig trajectoryConfig;
    private final SwerveSubsystem driveSubsystem = RobotContainer.swerveSubsystem;
    private final AutoFactory autoFactory;

    
    public Autos(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        // 1. Create trajectory settings
        this.trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose, // A function that returns the current robot pose
            swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerveSubsystem::followPath, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
        );
    }

    
}
