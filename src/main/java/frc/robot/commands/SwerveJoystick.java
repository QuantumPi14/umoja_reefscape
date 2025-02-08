// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  Joystick j = new Joystick(USB.DRIVER_CONTROLLER);

  private final PIDController driftController = new PIDController(DriveConstants.kPDrift, 0, 0);


  /** Creates a new SwerveJoystick. */
  public SwerveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFuntion) {

      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFuntion;
  
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
    
    if (j.getRawButton(OIConstants.Y)) {
      double KpDistance = -0.1f;  // Proportional control constant for distance
      double current_distance = Estimate_Distance();  // see the 'Case Study: Estimating Distance'
            double tx = LimelightHelpers.getTX(LimelightConstants.tagName);
            double distance_error = tx;
              
            double ty = LimelightHelpers.getTY(LimelightConstants.tagName);
          
            double steering_adjust = 0.0f;
            if (ty == 0.0f)
            {
              // We don't see the target, seek for the target by spinning in place at a safe speed.
              steering_adjust = 0.3f;		
            }
            else
            {
              // We do see the target, execute aiming code
              if (j.getRawButton(OIConstants.X)) {
              Double heading_error = tx;
                  steering_adjust = KpDistance * tx;
                  double desired_distance = 10;
                  double distance_error2 = desired_distance - current_distance;
                  // driving_adjust = KpDistance * distance_error2;
                      
            }
          }
             
                
            
          NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
          NetworkTableEntry tv = table.getEntry("ty");
          double targetOffsetAngle_Vertical = tv.getDouble(0.0);
      
          // how many degrees back is your limelight rotated from perfectly vertical?
          double limelightMountAngleDegrees = 25.0; 
      
          // distance from the center of the Limelight lens to the floor
          double limelightLensHeightInches = 20.0; 
      
          // distance from the target to the floor
          double goalHeightInches = 60.0; 
      
          double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
          double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
      
          //calculate distance
          double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
      
            // left_command+=steering_adjust; 
            // right_command-=steering_adjust;
            
           
      
            return;
      }

      Boolean aButtonPressed = j.getRawButton(OIConstants.A);
      if (aButtonPressed) {
        if (RobotContainer.currentTrajectory == null) {
            RobotContainer.currentTrajectory = swerveSubsystem.getNearestTagTrajectory();
        }
        // double curTime = swerveSubsystem.timer.get();
        // var desiredState = RobotContainer.currentTrajectory.sample(curTime);
        // var desiredRotation = RobotContainer.currentTrajectory.getStates().get(RobotContainer.currentTrajectory.getStates().size() - 1).poseMeters.getRotation();
        
        var desiredState = RobotContainer.currentTrajectory.getStates().get(RobotContainer.currentTrajectory.getStates().size() - 1);
        var desiredRotation = desiredState.poseMeters.getRotation();
        var targetChassisSpeeds =
            swerveSubsystem.holonomicDriveController.calculate(swerveSubsystem.getPose(), desiredState, desiredRotation);
        var targetModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        swerveSubsystem.setModuleStates(targetModuleStates);
        RobotContainer.wantedAngle = -1;
        return;
      } else {
        RobotContainer.currentTrajectory = null;
      }
      
      
          // 1. Get joystic inputs
          double xSpeed = xSpdFunction.get();
          double ySpeed = ySpdFunction.get();
          double turningSpeed = turningSpdFunction.get();
      
          // 2. Apply deadband
          // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
          // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
          if (Math.abs(xSpeed) + Math.abs(ySpeed) < OIConstants.kDeadband) {
            xSpeed = 0.0;
            ySpeed = 0.0;
          }
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
          
          double joystickX = xSpeed;
          double joystickY = ySpeed;
          double joystickTurn = turningSpeed;
      
          // 3. Make the driving smoother
          if (RobotContainer.driverController.getRawButton(OIConstants.kDriverRB)){
            xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
          }else{
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
          }
      
          // set current angle
          if (RobotContainer.wantedAngle == -1) {
            RobotContainer.wantedAngle = swerveSubsystem.getHeading();
          }

          if (joystickTurn == 0) {
            // fix drift
            // if drifting
            if (RobotContainer.wantedAngle != swerveSubsystem.getHeading()) {
              // > 180 we want to go towards 0 but from negative to account for closest angle
              double currentAngle = swerveSubsystem.getHeading();
              double diff = Math.abs(currentAngle - RobotContainer.wantedAngle);
              if (diff > 180) {
                diff = 360 - diff;
                // since diff is > 180, it passes the 360-0 range
                // if we have 359 and want 1, we go positive, so no change
                // if we have 1 and want 359, we go negative, so multiple -1
                if (RobotContainer.wantedAngle > 180) {
                  diff = diff * -1;
                }
              } else {
                // if going from 5 to 10, do nothing
                // if going from 10 to 5, we go negative so multiple -1
                if (currentAngle > RobotContainer.wantedAngle) {
                  diff = diff * -1;
                }
              }

              turningSpeed = driftController.calculate(-diff, 0);
              
              // only fix drift when moving
              if (joystickX == 0 && joystickY == 0) {
                turningSpeed = 0;
              }
            }
          } else {
            RobotContainer.wantedAngle = swerveSubsystem.getHeading();
          }

          // 4. Construct desired chassis speeds
          ChassisSpeeds chassisSpeeds;
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
      

          // 5. Convert chassis speeds to individual module states
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      
          swerveSubsystem.setModuleStates(moduleStates);
          
      
          // if(j.getRawButton(OIConstants.START)){
          //   swerveSubsystem.resetTurn();
          // }
          if(j.getRawButton(OIConstants.BACK)){
            swerveSubsystem.zeroHeading();
          }
        }
      
        private double Estimate_Distance() {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'Estimate_Distance'");
        }
      
        // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
