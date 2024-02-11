package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.subsystems.ChassisSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final XboxController xboxController = new XboxController(Constants.OperatorConstants.kXboxPort);
  CommandXboxController cmdXboxController = new CommandXboxController(Constants.OperatorConstants.kXboxPort);

  public RobotContainer() {
    this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
    ()-> -xboxController.getLeftY(),
    ()-> -xboxController.getLeftX(),
    ()-> -xboxController.getRightX()));
    configureBindings();
  }

  private void configureBindings() {
    // Applying zero heading method instant command to start button trigger
    cmdXboxController.start().onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));

    cmdXboxController.rightTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower)));
    cmdXboxController.rightTrigger().onFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));

    cmdXboxController.leftTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.collect()));
    cmdXboxController.leftTrigger().onFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
