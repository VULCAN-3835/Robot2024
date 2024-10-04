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

public class AutoShootCollectForwardCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCollectShootCmd. */
  public AutoShootCollectForwardCmd(ShooterSubsystem shooter, IntakeSubsystem intake, ChassisSubsystem chassis) {
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

      /** 5. Drive the chassis forward at full speed (1) */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)),

      /** 6. Wait for 2 seconds while moving forward */
      new WaitCommand(2),

      /** 7. Stop the chassis after the wait */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 8. Reset the intake state and close the intake arm */
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.restState);  // Switch the intake subsystem to the resting state
        intake.setRotationPosition(IntakeConstants.kClosedRotations);  // Move the intake arm to the closed position
      })
    );
  }
}
