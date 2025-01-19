// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  Joystick j = new Joystick(USB.DRIVER_CONTROLLER);

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

      PIDController pid = new PIDController(KpDistance, tx, distance_error)

      return;
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
