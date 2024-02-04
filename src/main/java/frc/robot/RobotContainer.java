package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.STATE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final XboxController xboxController = new XboxController(Constants.OperatorConstants.kXboxPort);

  public RobotContainer() {
    this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
    ()-> -xboxController.getLeftY(),
    ()-> -xboxController.getLeftX(),
    ()-> -xboxController.getRightX()));
    configureBindings();
  }

  private void configureBindings() {
    // Initilizing a start button trigger
    Trigger startTrigger = new Trigger(() -> this.xboxController.getStartButtonPressed());
    Trigger rightBumperTrigger = new Trigger(() -> this.xboxController.getRightBumper());
    Trigger leftBumperTrigger = new Trigger(() -> this.xboxController.getLeftBumper());
    Trigger yTrigger = new Trigger(() -> this.xboxController.getYButtonPressed());
    Trigger aTrigger = new Trigger(() -> this.xboxController.getAButtonPressed());

    // Applying zero heading method instant command to start button trigger
    startTrigger.onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));

    // Applying intake to intake motor on right bumper
    rightBumperTrigger.whileTrue(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.collectState)));
    rightBumperTrigger.onFalse(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.restState)));

    // Applying output to intake motor on left bumper
    leftBumperTrigger.whileTrue(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.outputState)));
    leftBumperTrigger.onFalse(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.restState)));

    // Applies positions open and closed buttons on y and a buttons.
    yTrigger.onTrue(new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kOpenAngle)));
    aTrigger.onTrue(new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedAngle)));

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
