package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class CollectCmd extends SequentialCommandGroup {

  /** Creates a new CollectCmd. */
  public CollectCmd(IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      /** 
       * 1. Activate the intake to start collecting game pieces.
       * Set the intake motor to the "collect" mode, set the shooter to run at the collection power,
       * and update the LED state to indicate the collection process is happening.
       */
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.collectState);  // Set intake to collect mode
        shooter.setShooterSpeed(ShooterConstants.kCollectPower);  // Set the shooter to a low power for collecting
        LEDController.setActionState(LEDController.ActionStates.SOURCE_COLLECTING);  // Change LEDs to indicate the robot is collecting
      }),

      /** 
       * 2. Wait until the intake detects that it has collected a piece.
       * The `hasPiece()` method checks if a game piece is present in the intake.
       */
      new WaitUntilCommand(() -> intake.hasPiece()),  // Wait for the intake sensor to detect a game piece

      /** 
       * 3. Once a piece is collected, stop the intake and shooter, and reset the LEDs to their default state.
       * This ensures the robot ceases collection and goes into a neutral state after the piece is secured.
       */
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.restState);  // Set the intake back to resting state (idle)
        shooter.setShooterSpeed(0);  // Stop the shooter motor
        LEDController.setActionState(LEDController.ActionStates.DEFAULT);  // Reset the LEDs back to their default state
      })
    );
  }
}
