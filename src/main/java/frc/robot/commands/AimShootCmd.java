package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShootCmd extends SequentialCommandGroup {

  /** Creates a new AimShootCmd. */
  public AimShootCmd(ChassisSubsystem chassis, IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<Boolean> backButton) {
    addCommands(
      /** 1. Change the LED state to indicate that the robot is aiming at the target */
      new InstantCommand(() -> LEDController.setActionState(LEDController.ActionStates.SPEAKER_AIMING)),
      
      /**
       *  2. Aim the chassis towards a specific AprilTag (either tag 7 or tag 4 depending on the alliance color).
       * If the alliance color is blue, aim at tag 7; if it's another color, aim at tag 4.
       * The 'backButton' supplier is used to possibly abort the command
      */
      new AimAtAprilTagCmd(chassis, Robot.allianceColor == "BLUE" ? 7 : 4, backButton),

      /**
      * 3. Shoot using the shooter and intake subsystems.
      * The 'ShootCmd' uses both subsystems to fire a game piece towards the target.
      */
      new ShootCmd(shooter, intake));
  }
}
