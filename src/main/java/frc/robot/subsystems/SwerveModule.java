// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
  private final TalonFX turnMotorFx;
  private final TalonFX driveMotorFX; 
  private final PIDController turningPidController;
  private final CANcoder encoder;

  public SwerveModule(int turnMotor, int driveMotor, int encoder, boolean driveReversed, boolean turnReversed) {
    this.turnMotorFx = new TalonFX(turnMotor);
    this.driveMotorFX = new TalonFX(driveMotor);
    this.encoder = new CANcoder(encoder);
    
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = Constants.ServeConstants.defaultStatorLimit;
    currentLimits.StatorCurrentLimitEnable = true;
      
    TalonFXConfigurator turnConfigurator = turnMotorFx.getConfigurator();
    TalonFXConfigurator driveConfigurator = turnMotorFx.getConfigurator();

  
    if(turnReversed) {
      MotorOutputConfigs turnOutput = new MotorOutputConfigs();
      turnOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      turnConfigurator.apply(turnOutput);
    }

    if(driveReversed) {
      MotorOutputConfigs driveOutput = new MotorOutputConfigs();
      driveOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      driveConfigurator.apply(driveOutput);
    }
    
    driveConfigurator.apply(currentLimits);
    turnConfigurator.apply(currentLimits);

    turningPidController = new PIDController(0.008, 0,0);
    turningPidController.enableContinuousInput(-180, 180);
    driveMotorFX.setNeutralMode(NeutralModeValue.Coast);
    turnMotorFx.setNeutralMode(NeutralModeValue.Coast);
  }

  public SwerveModulePosition getDriveInMeter() {
    double motor = driveMotorFX.getRotorPosition().getValueAsDouble() / 5.36;
    return new SwerveModulePosition(motor * 0.319, new Rotation2d(getTurningPosition()*(Math.PI/180))); 
  }

  public double getDrivePosition() 
    {
        return driveMotorFX.getPosition().getValueAsDouble();
    }

    public double getTurningPosition() 
    {
        return encoder.getAbsolutePosition().getValueAsDouble()*360;
    }
    

    public double getDriveVelocity() 
    {
        return driveMotorFX.getVelocity().getValueAsDouble();
    }

    public void resetEncoders() {

        driveMotorFX.setPosition(0);
        encoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()*(Math.PI/180)));   
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()*(Math.PI/180)));
    }

    public void setDesiredState(SwerveModuleState state) 
    {
        state.optimize(getState().angle);
        
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        driveMotorFX.set(state.speedMetersPerSecond/Constants.ServeConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotorFx.set(turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void setSpeedTurn(double speed)
    {
      turnMotorFx.set(speed);
    }

    public void setSpeedDrive(double speed)
    {
        driveMotorFX.set(speed);
    }

    public void stop() 
    {
        driveMotorFX.set(0); 
        turnMotorFx.set(0);
    }

    public void setToAngle(double angle)
    {
        turnMotorFx.setPosition(turningPidController.calculate(getTurningPosition(), angle));
    }
}
