package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class AmpShootCmd extends SequentialCommandGroup {

  /** Creates a new AmpShootCmd. */
  public AmpShootCmd(IntakeSubsystem intake) {
    addCommands(
      /** 1. Set the intake's rotation position to a target position and update LED state to indicate aiming */
      new InstantCommand(() -> {
          intake.setRotationPosition(0.41);  // Moves the intake arm to the desired angle for shooting
          LEDController.setActionState(LEDController.ActionStates.AMP_AIMING);  // Change LEDs to indicate the aiming process
      }),
      
      /** 2. Wait for 0.2 seconds to give the intake time to move to the position */
      new WaitCommand(0.2),  // Pause for 200 milliseconds to allow the intake mechanism to rotate

      /** 3. Wait until the intake arm reaches its set position */
      new WaitUntilCommand(() -> intake.getArmAtSetpoint()),  // Continue only when the arm reaches the specified rotation

      /** 4. Activate the intake motor in shooting mode and update LED state to indicate shooting */
      new InstantCommand(() -> {
          intake.setMotorMode(INTAKE_STATE.ampState);  // Switch the intake to the amp shooting state (powerful intake mode)
          LEDController.setActionState(LEDController.ActionStates.AMP_SHOOTING);  // Change LEDs to indicate the shooting process
      }),

      /** 5. Wait for 0.8 seconds to allow time for the shot to complete */
      new WaitCommand(0.8),  // Wait for the shooting action to complete (adjust timing as needed)

      /** 6. Return the intake motor and arm to their resting states, and reset the LEDs to their default state */
      new InstantCommand(() -> {
       intake.setMotorMode(INTAKE_STATE.restState);  // Return the intake motor to the resting state (idle)
       intake.setRotationPosition(IntakeConstants.kClosedRotations);  // Retract the intake arm to its closed position
       LEDController.setActionState(LEDController.ActionStates.DEFAULT);  // Reset the LEDs back to their default state
      })
    );
  }
}
