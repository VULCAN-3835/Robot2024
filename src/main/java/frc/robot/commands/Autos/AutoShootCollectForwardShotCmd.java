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
    // Declare the chassis subsystem as a requirement for this command group.
    addRequirements(chassis);
    
    addCommands(
      /** 1. Execute the auto shoot and collect command  */
      new AutoShootCollectForwardCmd(shooter,intake,chassis),

      /** 2. Drive the chassis backward at a speed of -1 for 1.85 seconds */
      new InstantCommand(() -> chassis.drive(-1, 0, 0, false)),
      new WaitCommand(1.85),

      /** 3. Stop the chassis after the backward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 4. Wait for 1 second before shooting again */
      new WaitCommand(1),

      /** 5. Execute the shooting command again */
      new ShootCmd(shooter, intake),

      /** 6. Wait for 1 second after shooting */
      new WaitCommand(1),

      /** 7. Drive the chassis forward at a speed of 1 for 3 seconds */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)),
      new WaitCommand(3),

      /** 8. Stop the chassis after the forward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false))
    );
  }
}
