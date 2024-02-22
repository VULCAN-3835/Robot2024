// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ChassisSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtAprilTagCmd extends PIDCommand {
  ChassisSubsystem chassisSubsystem;
  private double rotTolerance = 1;
  private Supplier<Boolean> backSupplier;
  private boolean wrongId = false;
  public AimAtAprilTagCmd(ChassisSubsystem chassisSubsystem, int id, Supplier<Boolean> backButton) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0, 0),
        // This should return the measurement
        () -> chassisSubsystem.getLimelight().getX(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          chassisSubsystem.drive(0, 0, output, true);// Use the output here
        });
    wrongId = false;
    this.chassisSubsystem = chassisSubsystem;
    wrongId = (double)id != this.chassisSubsystem.getLimelight().getAprilTagID();
    getController().setTolerance(this.rotTolerance);
    this.backSupplier = backButton;
    this.chassisSubsystem = chassisSubsystem;
    addRequirements(this.chassisSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Wrong ID", wrongId);
    SmartDashboard.putNumber("ID",this.chassisSubsystem.getLimelight().getAprilTagID());
    return backSupplier.get() || getController().atSetpoint() || wrongId;
  }
}
