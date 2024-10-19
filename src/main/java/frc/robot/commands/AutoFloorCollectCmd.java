// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class AutoFloorCollectCmd extends Command {
  private final ChassisSubsystem chassisSubsystem;  // Reference to the robot's chassis subsystem for driving control
  private final IntakeSubsystem intakeSubsystem;    // Reference to the intake subsystem for piece collection
  private final PIDController rotationPID;          // PID controller for managing rotation based on camera input
  private Supplier<Boolean> backSupplier;           // Supplier to check for back button or trigger state
  private boolean inTolerence;                       // Flag to check if the robot is within tolerance for rotation

  private final double kRotationPidKp = 0.05;      // Proportional gain for PID controller
  private final double kRotationPidKi = 0;         // Integral gain for PID controller (not used)
  private final double kRotationPidKd = 0;         // Derivative gain for PID controller (not used)
  private final double kDefaultFwdDriveSpeed = 1.5; // Default forward driving speed when within tolerance

  /** Creates a new FloorIntakeCommand. */
  public AutoFloorCollectCmd(ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Boolean> backSupplier) {
    this.chassisSubsystem = chassis;               // Initialize chassis subsystem
    this.intakeSubsystem = intake;                 // Initialize intake subsystem

    this.rotationPID = new PIDController(kRotationPidKp, kRotationPidKi, kRotationPidKd); // Initialize PID controller
    this.backSupplier = backSupplier;               // Initialize back supplier
    this.inTolerence = false;                       // Initialize tolerance flag
    addRequirements(chassisSubsystem, intakeSubsystem); // Declare subsystem requirements for the command
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.rotationPID.setTolerance(2); // Set the tolerance for the PID controller
    this.rotationPID.setSetpoint(0); // Set desired setpoint for rotation based on camera input

    this.intakeSubsystem.setMotorMode(INTAKE_STATE.collectState); // Set intake to collecting state
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rotationPID.atSetpoint()) { // Check if the robot is within the setpoint tolerance
      inTolerence = true; // Update tolerance flag if at setpoint
    }
    // Calculate the rotational speed using the PID controller if the intake is open, otherwise set to 0
    double rotSpeed = rotationPID.calculate(intakeSubsystem.getLimelight().getX());

    // Set forward speed to the default if in tolerance, otherwise set to 0
    double fwdSpeed = inTolerence ? kDefaultFwdDriveSpeed : 0; 

    // Drive the chassis with the calculated forward and rotational speeds
    chassisSubsystem.drive(fwdSpeed, 0, rotSpeed, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(0, 0, 0, false); // Stop the robot's movement
    intakeSubsystem.setMotorMode(IntakeSubsystem.INTAKE_STATE.restState); // Set the intake motor to the resting state

    // Reset flags
    inTolerence = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeSubsystem.hasPiece()) || backSupplier.get(); // End the command if a piece is collected or the back button is pressed
  }
}
