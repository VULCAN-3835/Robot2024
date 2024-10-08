package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AimShootCmd;
import frc.robot.commands.AmpShootCmd;
import frc.robot.commands.CollectCmd;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.FullFloorIntakeCmd;
import frc.robot.commands.NormalCollectCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.Autos.AutoShootCmd;
import frc.robot.commands.Autos.AutoShootCollectForwardCmd;
import frc.robot.commands.Autos.AutoShootCollectForwardShotCmd;
import frc.robot.commands.Autos.AutoShootMoveCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final XboxController xboxControllerDrive = new XboxController(OperatorConstants.kXboxDrivePort);
  private final XboxController xboxControllerButton = new XboxController(OperatorConstants.kXboxButtonPort);
  private final Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  private final Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Autonomous chooser
    NamedCommands.registerCommand("ShootCmd", new ShootCmd(shooterSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("AimShootCmd", new AimShootCmd(chassisSubsystem, intakeSubsystem, shooterSubsystem, () -> false));
    NamedCommands.registerCommand("AutoCollect", new FullFloorIntakeCmd(chassisSubsystem, intakeSubsystem, () -> false));
    NamedCommands.registerCommand("AmpShootCmd", new AmpShootCmd(intakeSubsystem));
    NamedCommands.registerCommand("OpenIntake", new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kOpenRotations)));
    NamedCommands.registerCommand("CloseIntake", new InstantCommand(() -> this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations)));
    NamedCommands.registerCommand("ActivateShooter", new InstantCommand(() -> this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower)));

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Empty", null);
    autoChooser.addOption("Shoot", new AutoShootCmd(this.shooterSubsystem, this.intakeSubsystem));
    autoChooser.addOption("Shoot Move", new AutoShootMoveCmd(this.shooterSubsystem, this.intakeSubsystem, this.chassisSubsystem));
    autoChooser.addOption("Shoot Collect Forward", new AutoShootCollectForwardCmd(this.shooterSubsystem, this.intakeSubsystem,this.chassisSubsystem));
    autoChooser.addOption("Shoot Collect Forward Shoot",new AutoShootCollectForwardShotCmd(this.shooterSubsystem, this.intakeSubsystem, this.chassisSubsystem));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    LEDSubsystem.getInstance(); // just to build the singleton once
    LEDController.setActionState(LEDController.ActionStates.DEFAULT);
  }

  private void configureBindings() {
    setUpControllers();
  }

  public void setUpControllers() {
    if(xboxControllerDrive.isConnected()) {
      this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
      () -> -xboxControllerDrive.getLeftY(),
      () -> -xboxControllerDrive.getLeftX(),
      () -> -xboxControllerDrive.getRightX()));

      configureXboxBinding(OperatorConstants.kXboxDrivePort);

      if (xboxControllerButton.isConnected() && xboxControllerDrive.isConnected()) {
        configureXboxBinding(OperatorConstants.kXboxButtonPort);
      }
    }
    else {
      this.chassisSubsystem.setDefaultCommand(
      new DefaultTeleopCommand(this.chassisSubsystem,
      () -> -leftJoystick.getY(),
      () -> -leftJoystick.getX(),
      () -> -rightJoystick.getX()));
      configureXboxBinding(OperatorConstants.kXboxButtonPort);
    }
  }

  
  private void configureXboxBinding(int port) {
    CommandXboxController cmdXboxController = new CommandXboxController(port);
    
    // Applies zero heading method instant command to start button trigger
    cmdXboxController.start().onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));

    // RIGHT TRIGGER
    cmdXboxController.rightTrigger().whileTrue(new ShootCmd(shooterSubsystem, intakeSubsystem));
    cmdXboxController.rightTrigger().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.shooterSubsystem.setShooterSpeed(0);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));
    // LEFT TRIGGER
    cmdXboxController.leftTrigger().whileTrue(new NormalCollectCmd(this.intakeSubsystem));
    cmdXboxController.leftTrigger().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // A TRIGGER
    cmdXboxController.a().whileTrue(new FullFloorIntakeCmd(chassisSubsystem, intakeSubsystem, () -> cmdXboxController.back().getAsBoolean()));
    cmdXboxController.a().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // B Trigger
    cmdXboxController.b().whileTrue(new AmpShootCmd(intakeSubsystem));
    cmdXboxController.b().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // Y Trigger
    cmdXboxController.y().whileTrue(new AimShootCmd(chassisSubsystem, intakeSubsystem, shooterSubsystem, () -> cmdXboxController.back().getAsBoolean()));
    cmdXboxController.y().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.shooterSubsystem.setShooterSpeed(0);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // X Trigger
    cmdXboxController.x().whileTrue(new CollectCmd(intakeSubsystem, shooterSubsystem));
    cmdXboxController.x().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.shooterSubsystem.setShooterSpeed(0);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // POV UP Trigger
    cmdXboxController.povUp().whileTrue(new InstantCommand(() -> {
      this.climberSubsystem.setMotorsPowers(ClimberConstants.kClimbUpPower);
      LEDController.setActionState(LEDController.ActionStates.OPENING_CLIMBER);
    }));
    cmdXboxController.povUp().toggleOnFalse(new InstantCommand(() -> {
      this.climberSubsystem.setMotorsPowers(0);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    cmdXboxController.povDown().whileTrue(new InstantCommand(() -> {
      this.climberSubsystem.setMotorsPowers(ClimberConstants.kClimbDownPower);
      LEDController.setActionState(LEDController.ActionStates.OPENING_CLIMBER);
    }));
    cmdXboxController.povDown().toggleOnFalse(new InstantCommand(() -> {
      this.climberSubsystem.setMotorsPowers(0);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    Command auto = autoChooser.getSelected();
    return auto;
  }
}
