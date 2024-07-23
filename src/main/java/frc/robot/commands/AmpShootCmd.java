// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class AmpShootCmd extends SequentialCommandGroup {

  /** Creates a new AmpShootCmd. */
  public AmpShootCmd(IntakeSubsystem intake) {
    addCommands(
      new InstantCommand(() -> {
          intake.setRotationPosition(0.41);
          LEDController.setActionState(LEDController.ActionStates.AMP_AIMING);
      }),
      new WaitCommand(0.2),
      new WaitUntilCommand(() -> intake.getArmAtSetpoint()),
      new InstantCommand(() -> {
          intake.setMotorMode(INTAKE_STATE.ampState);
          LEDController.setActionState(LEDController.ActionStates.AMP_SHOOTING);
      }),
      new WaitCommand(0.8),
      new InstantCommand(() -> {
       intake.setMotorMode(INTAKE_STATE.restState);
       intake.setRotationPosition(IntakeConstants.kClosedRotations);
       LEDController.setActionState(LEDController.ActionStates.DEFAULT);
      })
    );
  }
}
