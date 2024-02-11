package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.STATE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController xboxController = new XboxController(OperatorConstants.kXboxPort);
  private final CommandXboxController cmdXboxController = new CommandXboxController(OperatorConstants.kXboxPort);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
    ()-> -xboxController.getLeftY(),
    ()-> -xboxController.getLeftX(),
    ()-> -xboxController.getRightX()));
    configureBindings();
  }

  private void configureBindings() {
    // Initilizing a start button trigger
    Trigger rightBumperTrigger = new Trigger(() -> this.xboxController.getRightBumper());
    Trigger leftBumperTrigger = new Trigger(() -> this.xboxController.getLeftBumper());


    // Applying zero heading method instant command to start button trigger
    cmdXboxController.start().onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));
    
    // Applying intake to intake motor on right bumper
    rightBumperTrigger.whileTrue(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.outputState)));
    rightBumperTrigger.onFalse(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.restState)));

    cmdXboxController.rightTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower)));
    cmdXboxController.rightTrigger().onFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));

    cmdXboxController.leftTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.collect()));
    cmdXboxController.leftTrigger().onFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));

    // Applying output to intake motor on left bumper
    leftBumperTrigger.whileTrue(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.collectState)));
    leftBumperTrigger.onFalse(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.restState)));

    // Applies positions open and closed buttons on y and a buttons.
    cmdXboxController.a().onTrue(new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kOpenRotations)));
    cmdXboxController.y().onTrue(new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations)));

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
