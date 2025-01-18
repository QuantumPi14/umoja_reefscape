package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    public final TalonFX driveMotor, turnMotor;
    // private final RelativeEncoder driveEncoder, turnEncoder;
    private final PIDController turnPIDController;
    // public final CANcoder absoluteEncoder;
    public double absoluteEncoderOffset;
    public final AnalogInput absoluteEncoder;
    public final int absoluteEncoderID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed){   
        // CANcoderConfiguration config = new CANcoderConfiguration();
        
        // config.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        // absoluteEncoder.setVoltagePercentageRange(absoluteEncoderId, absoluteEncoderOffset);
        absoluteEncoder.setInverted(isAbsoluteEncoderReversed);
        // absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
        this.absoluteEncoderID = absoluteEncoderId;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new TalonFX(turnMotorId);

        // driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        // turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

        // MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        // motorConfigs.withInverted(InvertedValue.Clockwise_Positive);
        // driveMotor.getConfigurator().apply(motorConfigs);
        
        // driveMotor.setInverted(false);
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        
        // driveEncoder = driveMotor.getEncoder(Type.kHallSensor, 42);
        // turnEncoder = turnMotor.getEncoder(Type.kHallSensor, 42);

        // driveMotor.setSmartCurrentLimit(30);
        // turnMotor.setSmartCurrentLimit(20);


        // driveMotor.set
        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        // turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        turnPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getTurningPosition() {
        return turnMotor.getPosition().getValueAsDouble()*ModuleConstants.kTurnEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurningVelocity() {
        return turnMotor.getVelocity().getValueAsDouble();
    }

    public double 
    getAbsoluteEncoderRad() {
        return absoluteEncoder.get() - absoluteEncoderOffset;   
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        resetTurn();
    }

    public void resetTurn(){
        double position = getAbsoluteEncoderRad();
        turnMotor.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // driveMotor.set(drivePIDcontroller.calculate())
        // TODO: CHANGE THIS TO PID

        //fill canbus
        // SmartDashboard.putNumber("ID (DRIVE) "+absoluteEncoderID + " TEMP: ", driveMotor.getMotorTemperature());
        // SmartDashboard.putNumber("ID (TURN) "+absoluteEncoderID + " TEMP: ", turnMotor.getMotorTemperature());

        // SmartDashboard.putNumber("ID: " + absoluteEncoderID, Math.toDegrees(getTurningPosition()));
        // SmartDashboard.putNumber("GOAL: " + absoluteEncoderID, Math.toDegrees(state.angle.getRadians()));
        // SmartDashboard.putNumber("Set motor percent: " + absoluteEncoderID, turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        
        turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        //turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    // public void resetAbsoluteEncoders(){
    //     absoluteEncoder.resetPosition();
    //     System.out.println("Reset Absolute Encoder " + absoluteEncoderID);
    // }
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble(),
            Rotation2d.fromRadians(getTurningPosition()));
      }
}