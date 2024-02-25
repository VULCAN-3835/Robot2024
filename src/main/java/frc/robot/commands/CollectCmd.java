// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectCmd extends SequentialCommandGroup {
  /** Creates a new CollectCmd. */
  public CollectCmd(IntakeSubsystem intake, ShooterSubsystem shooter) {
    
    addCommands(
      new InstantCommand(() -> {
       intake.setMotorMode(INTAKE_STATE.collectState);
       shooter.setShooterSpeed(ShooterConstants.kCollectPower); 
      }),
      new WaitUntilCommand(() -> intake.hasPiece()),
      new InstantCommand(() -> {
      intake.setMotorMode(INTAKE_STATE.restState);
      shooter.setShooterSpeed(0); 
      })
    );
  }
}
