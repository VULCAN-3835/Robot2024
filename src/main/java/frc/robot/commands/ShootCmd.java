package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LEDController;
import frc.robot.Util.LEDController.ActionStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends SequentialCommandGroup {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  /** Creates a new ShootCmd. */
  public ShootCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem; // Reference to the shooter subsystem
    this.intakeSubsystem = intakeSubsystem; // Reference to the intake subsystem

    addRequirements(this.shooterSubsystem, this.intakeSubsystem); // Declare subsystem requirements

    addCommands(
      /** 1. Set the intake motor to collect state */
      new InstantCommand(() -> this.intakeSubsystem.setMotorMode(INTAKE_STATE.collectState)),
      
      /** 2. Wait for a short duration to allow the intake to prepare */
      new WaitCommand(0.1), 
      
      /** 3. Set the intake motor to resting state after preparing */
      new InstantCommand(() -> this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState)),
      
      /** 4. Set the shooter speed to the defined shoot power and update the LED state */
      new InstantCommand(() -> {
        this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower); // Activate shooter at shooting power
        LEDController.setActionState(ActionStates.SPEAKER_SHOOTING); // Update LEDs to indicate shooting action
      }),
      
      /** 5. Wait for a short duration to allow the shooter to reach speed */
      new WaitCommand(0.1), 
      
      /** 6. Wait until the shooter reaches the specified RPM threshold */
      new WaitUntilCommand(() -> (Math.abs(shooterSubsystem.getShooterSpeedRPM()) >= 3800)),
      
      /** 7. Set the intake motor to output state to eject the game piece */
      new InstantCommand(() -> {
        this.intakeSubsystem.setMotorMode(INTAKE_STATE.outputState); // Switch the intake to output mode
        LEDController.setActionState(ActionStates.SPEAKER_SHOOTING); // Keep LEDs indicating shooting
      }),
      
      /** 8. Wait for a short duration to allow the piece to eject */
      new WaitCommand(0.2),
      
      /** 9. Stop the shooter motor and reset LED state */
      new InstantCommand(() -> {
        this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
        this.shooterSubsystem.stopMotor(); // Stop the shooter after shooting is complete
        LEDController.setActionState(ActionStates.DEFAULT); // Reset LEDs back to default state
    })
    );
  }
}