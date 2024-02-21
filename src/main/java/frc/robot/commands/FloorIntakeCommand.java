// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FloorIntakeCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PIDController rotationPID;
  private Supplier<Boolean> backSupplier;
  private boolean finished;
  private boolean inTolerence;


  /** Creates a new FloorIntakeCommand. */
  public FloorIntakeCommand(ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Boolean> backSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassis;
    this.intakeSubsystem = intake;
    addRequirements(chassisSubsystem, intakeSubsystem);

    this.rotationPID = new PIDController(Constants.CommandConstants.kRotationPidKp, Constants.CommandConstants.kRotationPidKi, Constants.CommandConstants.kRotationPidKd);
    this.backSupplier = backSupplier;
    this.finished = false;
    this.inTolerence = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.rotationPID.setTolerance(2);
    this.rotationPID.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rotationPID.atSetpoint()) {
      inTolerence = true;
    }

    if (intakeSubsystem.hasPiece()) {
      if (intakeSubsystem.isOpen()) {
        this.intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.restState);
        this.chassisSubsystem.drive(0,0,0, false);
        this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
      }
    if (this.intakeSubsystem.isClosed())
        finished = true;
    }
    else {
      double rotSpeed = intakeSubsystem.isOpen()?rotationPID.calculate(intakeSubsystem.getPieceX()):0; //PID output for rot
      double fwdSpeed = inTolerence ? Constants.CommandConstants.kDefaultFwdDriveSpeed : 0;
      if (rotationPID.atSetpoint())
        intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.collectState);
      else 
        intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.restState);

      chassisSubsystem.drive(fwdSpeed, 0, rotSpeed, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(0, 0, 0, false);//Stops The Robot
    intakeSubsystem.setMotorMode(IntakeSubsystem.STATE.restState);

    // Reset flags
    finished = false;
    inTolerence = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished || backSupplier.get(); //Or trigger released
  }
}
