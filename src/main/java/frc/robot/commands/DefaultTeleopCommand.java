// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ChassisSubsystem;

public class DefaultTeleopCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private boolean fieldOriented;
  public DefaultTeleopCommand(ChassisSubsystem chassisSubsystem, Supplier<Double> xSpdFunction,
    Supplier<Double> ySpdFunction,
    Supplier<Double> turningSpdFunction) {

    this.chassisSubsystem = chassisSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    this.xLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.yLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.turningLimiter = new SlewRateLimiter(Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);

    fieldOriented = false;
    
    addRequirements(this.chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.kDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.kDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > Constants.OperatorConstants.kDeadband ? turningSpeed : 0;

    xSpeed = xLimiter.calculate(xSpeed) * Constants.ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;
    turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec;

    this.chassisSubsystem.drive(xSpeed, ySpeed, turningSpeed, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
