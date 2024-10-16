// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ChassisSubsystem;

public class AimAtAprilTagCmd extends PIDCommand {
  ChassisSubsystem chassisSubsystem;
  private double rotTolerance = 1;
  private Supplier<Boolean> backSupplier;
  private boolean wrongId = false;

  /** Creates a new AimAtAprilTagCmd. */
  public AimAtAprilTagCmd(ChassisSubsystem chassisSubsystem, int aprilTagId, Supplier<Boolean> backButton) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0, 0),
        // This should return the measurement
        () -> chassisSubsystem.getLimelight().getX(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          chassisSubsystem.drive(0, 0, output, true);// Use the output here
        });

    // Set flags, refrences and tolerance
    wrongId = false;
    this.chassisSubsystem = chassisSubsystem;
    wrongId = (double)aprilTagId != this.chassisSubsystem.getLimelight().getAprilTagID();
    getController().setTolerance(this.rotTolerance);
    this.backSupplier = backButton;
    this.chassisSubsystem = chassisSubsystem;

    SmartDashboard.putNumber("Got ID", aprilTagId);

    addRequirements(this.chassisSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Wrong ID", wrongId);
    SmartDashboard.putNumber("ID",this.chassisSubsystem.getLimelight().getAprilTagID());

    /** 
    * The command ends if any of the following conditions are met:
    * - The back button is pressed
    * - The PID controller has reached the setpoint
    * - The detected April Tag ID is not the expected one
    */
    return backSupplier.get() || getController().atSetpoint() || wrongId;
  }
}
