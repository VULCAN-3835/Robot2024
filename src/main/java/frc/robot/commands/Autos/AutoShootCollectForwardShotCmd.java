// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCollectForwardShotCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCollectForwardShotCmd. */
  public AutoShootCollectForwardShotCmd(ShooterSubsystem shooter, IntakeSubsystem intake, ChassisSubsystem chassis) {
    addRequirements(chassis);
    
    addCommands(
      new ShootCmd(shooter, intake),
      new InstantCommand(() -> intake.setRotationPosition(IntakeConstants.kOpenRotations)),
      new WaitUntilCommand(() -> intake.isOpen()),
      new InstantCommand(() -> intake.setMotorMode(INTAKE_STATE.collectState)),
      new InstantCommand(() -> chassis.drive(1.2,0,0, false)),
      new WaitCommand(1.66),
      new InstantCommand(() -> chassis.drive(0,0,0, false)),
      new InstantCommand(() -> {
       intake.setMotorMode(INTAKE_STATE.restState);
       intake.setRotationPosition(IntakeConstants.kClosedRotations); 
      }),
      new InstantCommand(() -> chassis.drive(-1,0,0, false)),
      new WaitCommand(1.85),
      new InstantCommand(() -> chassis.drive(0,0,0, false)),
      new WaitCommand(1),
      new ShootCmd(shooter, intake),
      new WaitCommand(1),
      new InstantCommand(() -> chassis.drive(1,0,0, false)),
      new WaitCommand(3),
      new InstantCommand(() -> chassis.drive(0,0,0, false))
      );
  }
}
