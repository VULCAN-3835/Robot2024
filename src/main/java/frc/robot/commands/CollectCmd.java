// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class CollectCmd extends SequentialCommandGroup {

  /** Creates a new CollectCmd. */
  public CollectCmd(IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.collectState);
        shooter.setShooterSpeed(ShooterConstants.kCollectPower);
        LEDController.setActionState(LEDController.ActionStates.SOURCE_COLLECTING);
      }),
      new WaitUntilCommand(() -> intake.hasPiece()),
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.restState);
        shooter.setShooterSpeed(0);
        LEDController.setActionState(LEDController.ActionStates.DEFAULT);
      })
    );
  }
}
