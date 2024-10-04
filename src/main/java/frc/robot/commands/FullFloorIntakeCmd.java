package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;

public class FullFloorIntakeCmd extends SequentialCommandGroup {

  /** Creates a new FullFloorIntakeCmd. */
  public FullFloorIntakeCmd(ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Boolean> cancelButton) {
    addCommands(
      /** 1. Open the intake arm to the designated open position */
      new InstantCommand(() -> intake.setRotationPosition(IntakeConstants.kOpenRotations)),

      /** 2. Wait until the intake arm is fully open */
      new WaitUntilCommand(() -> intake.isOpen()),

      /** 3. Set the intake motor to collect mode and update LED state to indicate collecting */
      new InstantCommand(() -> {
        intake.setMotorMode(INTAKE_STATE.collectState);  // Switch the intake subsystem to the collecting state
        LEDController.setActionState(LEDController.ActionStates.FLOOR_COLLECTING);  // Change LEDs to indicate floor collecting
      }),

      /** 4. Wait until the Limelight camera detects a target */
      new WaitUntilCommand(() -> intake.getLimelight().cameraHasTarget()),

      /** 5. Execute the FloorIntakeCommand for further collection actions */
      new FloorIntakeCommand(chassis, intake, cancelButton)  // Proceed with the FloorIntakeCommand to manage the collection process
    );
  }
}
