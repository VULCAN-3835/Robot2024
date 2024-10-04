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
      /** 1. Execute the shooting command to shoot the game piece */
      new ShootCmd(shooter, intake),

      /** 2. Open the intake arm to the designated open position */
      new InstantCommand(() -> intake.setRotationPosition(IntakeConstants.kOpenRotations)),

      /** 3. Wait until the intake arm is fully open */
      new WaitUntilCommand(() -> intake.isOpen()),

      /** 4. Set the intake motor to collect mode */
      new InstantCommand(() -> intake.setMotorMode(INTAKE_STATE.collectState)),

      /** 5. Drive the chassis forward at a speed of 1.2 for 1.66 seconds */
      new InstantCommand(() -> chassis.drive(1.2, 0, 0, false)),
      new WaitCommand(1.66),

      /** 6. Stop the chassis after the wait */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 7. Reset the intake state and close the intake arm */
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.restState);  // Switch the intake subsystem to the resting state
        intake.setRotationPosition(IntakeConstants.kClosedRotations);  // Move the intake arm to the closed position
      }),

      /** 8. Drive the chassis backward at a speed of -1 for 1.85 seconds */
      new InstantCommand(() -> chassis.drive(-1, 0, 0, false)),
      new WaitCommand(1.85),

      /** 9. Stop the chassis after the backward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 10. Wait for 1 second before shooting again */
      new WaitCommand(1),

      /** 11. Execute the shooting command again */
      new ShootCmd(shooter, intake),

      /** 12. Wait for 1 second after shooting */
      new WaitCommand(1),

      /** 13. Drive the chassis forward at a speed of 1 for 3 seconds */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)),
      new WaitCommand(3),

      /** 14. Stop the chassis after the forward drive */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false))
    );
  }
}
