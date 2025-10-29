// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
              this.swerveSubsystem = swerveSubsystem;
              this.xSpdFunction = xSpdFunction;
              this.ySpdFunction = ySpdFunction;
              this.turningSpdFunction = turningSpdFunction;
              this.fieldOrientedFunction = fieldOrientedFunction;

              this.xLimiter = new SlewRateLimiter(DrivetrainConstants.ServeConstants.maxAccelerationUnitsPerSecond);
              this.yLimiter = new SlewRateLimiter(DrivetrainConstants.ServeConstants.maxAccelerationUnitsPerSecond);
              this.turningLimiter = new SlewRateLimiter(DrivetrainConstants.ServeConstants.maxAngularAccelerationUnitsPerSecond);

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get() / DrivetrainConstants.ChasisConstants.speedLimiter;
    double ySpeed = ySpdFunction.get() / DrivetrainConstants.ChasisConstants.speedLimiter;
    double turningSpeed = turningSpdFunction.get() / DrivetrainConstants.ChasisConstants.speedLimiter;

    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
    
    xSpeed = xLimiter.calculate(xSpeed) * DrivetrainConstants.ServeConstants.kPhysicalMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DrivetrainConstants.ServeConstants.kPhysicalMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DrivetrainConstants.ServeConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get())
        {
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
         else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        
    SwerveModuleState[] moduleStates = DrivetrainConstants.ServeConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.driverJoystick.getRawAxis(OIConstants.kDriverYAxis)) <= 0.1 && 
           Math.abs(RobotContainer.driverJoystick.getRawAxis(OIConstants.kDriverXAxis)) <= 0.1;
  }
}