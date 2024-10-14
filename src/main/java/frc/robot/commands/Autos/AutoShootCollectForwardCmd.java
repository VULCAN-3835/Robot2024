package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

      /** 2. Set the intake motor to collect mode */
      new InstantCommand(() -> intake.setMotorMode(INTAKE_STATE.collectState)),

      /** 3. Drive the chassis forward at full speed (1) */
      new InstantCommand(() -> chassis.drive(1, 0, 0, false)),

      /** 4. Wait for 2 seconds while moving forward */
      new WaitCommand(2),

      /** 5. Stop the chassis after the wait */
      new InstantCommand(() -> chassis.drive(0, 0, 0, false)),

      /** 6. Reset the intake state*/
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.restState);  // Switch the intake subsystem to the resting state
      })
    );
  }
}
