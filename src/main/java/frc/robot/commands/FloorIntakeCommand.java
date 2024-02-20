// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;

public class FloorIntakeCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PIDController rotationPID;
  private final PIDController distancePID;

  /** Creates a new FloorIntakeCommand. */
  public FloorIntakeCommand(ChassisSubsystem chassis, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassis;
    this.intakeSubsystem = intake;
    addRequirements(chassisSubsystem, intakeSubsystem);

    this.rotationPID = new PIDController(Constants.CommandConstants.kRotationPidKp, Constants.CommandConstants.kRotationPidKi, Constants.CommandConstants.kRotationPidKd);
    this.distancePID = new PIDController(Constants.CommandConstants.kDistancePidKp, Constants.CommandConstants.kDistancePidKi, Constants.CommandConstants.kDistancePidKd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setRotationPosition(IntakeConstants.kOpenRotations);
    intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.collectState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotSpeed = rotationPID.calculate(intakeSubsystem.getPieceX()); //PID output for rot
    double fwdSpeed = distancePID.calculate(intakeSubsystem.getPieceY()); //PID output for movement
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(fwdSpeed, 0, rotSpeed);
    chassisSubsystem.drive(chassisSpeeds, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(new ChassisSpeeds(0, 0, 0), false);//Stops The Robot
    intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.restState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.hasPiece();
  }
}
