// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class NormalCollectCmd extends SequentialCommandGroup {

  /** Creates a new NormalCollectCmd. */
  public NormalCollectCmd(IntakeSubsystem intake) {
    addCommands(
      /** 1. Open the intake arm to the designated open position */
      new InstantCommand(() -> intake.setRotationPosition(IntakeConstants.kOpenRotations)),

      /** 2. Wait until the intake arm is fully open */
      new WaitUntilCommand(() -> intake.isOpen()),

      /** 3. Set the intake motor to collect mode and update LED state to indicate collecting */
      new InstantCommand(() -> {
          intake.setMotorMode(INTAKE_STATE.collectState);  // Switch the intake subsystem to the collecting state
          LEDController.setActionState(LEDController.ActionStates.FLOOR_COLLECTING);  // Change LEDs to indicate floor collecting
      }),

      /** 4. Wait until the intake detects that it has a game piece */
      new WaitUntilCommand(() -> intake.hasPiece()),

      /** 5. Return the intake motor and arm to their resting states, and reset the LEDs to default */
      new InstantCommand(() -> {
          intake.setMotorMode(INTAKE_STATE.restState);  // Switch the intake motor to its resting state
          intake.setRotationPosition(IntakeConstants.kClosedRotations);  // Retract the intake arm to its closed position
          LEDController.setActionState(LEDController.ActionStates.DEFAULT);  // Reset the LEDs back to their default state
      })
    );
  }
}
