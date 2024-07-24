// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LEDController;
import frc.robot.Util.LEDController.ActionStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends SequentialCommandGroup {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  /** Creates a new ShootCmd. */
  public ShootCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(this.shooterSubsystem, this.intakeSubsystem);

    addCommands(
      new InstantCommand(() -> this.intakeSubsystem.setMotorMode(INTAKE_STATE.collectState)),
      new WaitCommand(0.1),
      new InstantCommand(() -> this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState)),
      new InstantCommand(() -> {
        this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower);
        LEDController.setActionState(ActionStates.SPEAKER_SHOOTING);
      }),
      new WaitCommand(0.1), 
      new WaitUntilCommand(() -> (Math.abs(shooterSubsystem.getShooterSpeedRPM()) >= 3800)),
      new InstantCommand(() -> {
        this.intakeSubsystem.setMotorMode(INTAKE_STATE.outputState);
        LEDController.setActionState(LEDController.ActionStates.SPEAKER_SHOOTING);
      }),
      new WaitCommand(0.2),
      new InstantCommand(() -> this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState)),
      new InstantCommand(() -> {
        this.shooterSubsystem.stopMotor();
        LEDController.setActionState(ActionStates.DEFAULT);
    })
    );
  }
}
