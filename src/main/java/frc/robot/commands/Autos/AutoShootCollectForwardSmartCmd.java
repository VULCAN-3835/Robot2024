package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFloorCollectCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCollectForwardSmartCmd extends SequentialCommandGroup {

  /** Creates a new AutoShootCollectShootCmd. */
  public AutoShootCollectForwardSmartCmd(ShooterSubsystem shooter, IntakeSubsystem intake, ChassisSubsystem chassis) {
    // Declare the chassis subsystem as a requirement for this command group.
    addRequirements(chassis);
    
    addCommands(
      /** 1. Execute the shooting command to shoot the game piece */
      new ShootCmd(shooter, intake),

      /** 2. collects the note using camera vision */
      new AutoFloorCollectCmd(chassis, intake, () -> false)
    );
  }
}
