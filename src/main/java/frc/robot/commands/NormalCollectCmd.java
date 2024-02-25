// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NormalCollectCmd extends SequentialCommandGroup {
  /** Creates a new NormalCollectCmd. */
  public NormalCollectCmd(IntakeSubsystem intake ) {
    
    addCommands(
      new InstantCommand(() -> intake.setRotationPosition(IntakeConstants.kOpenRotations)),
      new WaitUntilCommand(() -> intake.isOpen()),
      new InstantCommand(() -> intake.setMotorMode(INTAKE_STATE.collectState)),
      new WaitUntilCommand(() -> intake.hasPiece()),
      new InstantCommand(() -> {
       intake.setMotorMode(INTAKE_STATE.restState);
       intake.setRotationPosition(IntakeConstants.kClosedRotations); 
      })
    );
  }
}
