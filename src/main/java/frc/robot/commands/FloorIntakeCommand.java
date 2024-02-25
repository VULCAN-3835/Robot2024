// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class FloorIntakeCommand extends Command {
  private final ChassisSubsystem chassisSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PIDController rotationPID;
  private Supplier<Boolean> backSupplier;
  private boolean inTolerence;

  private final double kRotationPidKp = 0.045;
  private final double kRotationPidKi = 0;
  private final double kRotationPidKd = 0;
  private final double kDefaultFwdDriveSpeed = 1.2; //Find actual def speed

  /** Creates a new FloorIntakeCommand. */
  public FloorIntakeCommand(ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Boolean> backSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassis;
    this.intakeSubsystem = intake;

    this.rotationPID = new PIDController(kRotationPidKp, kRotationPidKi, kRotationPidKd);
    this.backSupplier = backSupplier;
    this.inTolerence = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.rotationPID.setTolerance(2);
    this.rotationPID.setSetpoint(0);

    this.intakeSubsystem.setMotorMode(INTAKE_STATE.collectState);

    addRequirements(chassisSubsystem, intakeSubsystem);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rotationPID.atSetpoint()) {
      inTolerence = true;
    }
    double rotSpeed = intakeSubsystem.isOpen()?rotationPID.calculate(intakeSubsystem.getLimelight().getX()):0; //PID output for rot
    double fwdSpeed = inTolerence ? kDefaultFwdDriveSpeed : 0;
    

    chassisSubsystem.drive(fwdSpeed, 0, rotSpeed, false);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(0, 0, 0, false);//Stops The Robot
    this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
    intakeSubsystem.setMotorMode(IntakeSubsystem.INTAKE_STATE.restState);

    // Reset flags
    inTolerence = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeSubsystem.hasPiece() && intakeSubsystem.isOpen())|| backSupplier.get(); //Or trigger released
  }
}
