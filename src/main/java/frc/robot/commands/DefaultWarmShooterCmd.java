// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultWarmShooterCmd extends Command {

    /** Creates a new DefaultTeleopCommand. */
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
  public DefaultWarmShooterCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    
    addRequirements(this.shooterSubsystem,this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    int aprilTagSee = (int) shooterSubsystem.getLimelightShooter().getAprilTagID();
    if((aprilTagSee == 3 || aprilTagSee == 4 || aprilTagSee == 7 || aprilTagSee == 8)
    && intakeSubsystem.hasPiece()){
      shooterSubsystem.setShooterSpeed(Constants.ShooterConstants.kWarmMotorPower);
    }
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
