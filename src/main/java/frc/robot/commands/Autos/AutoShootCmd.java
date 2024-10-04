package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCmd. */
  public AutoShootCmd(ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add the ShootCmd to the sequence of commands for this autonomous command
    addCommands(
      new ShootCmd(shooter, intake)  // Execute the ShootCmd, which manages the shooting process
    );
  }
}
