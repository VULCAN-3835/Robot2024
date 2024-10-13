// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ShooterConstants;


public class SourceCollectCmd extends SequentialCommandGroup {
  /** Creates a new SourceIntake. */
  public SourceCollectCmd(IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(//TODO: add LED controls

      //1. sets the power of the shooter and the intake to collect piece
      new InstantCommand(() -> {
        shooter.setShooterSpeed(ShooterConstants.kCollectPower);
        intake.setMotorMode(INTAKE_STATE.outputState);
      }),

      //2. collects until it has piece detected
      new WaitUntilCommand( ()-> intake.hasPiece()),
      
      //4. output the note until the sensor doesn't sense it anymore
      new WaitUntilCommand(() -> !intake.hasPiece()),

      //5. collects the note 
      new FloorCollectCmd(intake)
    );
  }
}
