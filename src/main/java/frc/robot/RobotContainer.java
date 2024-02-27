package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.INTAKE_STATE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.Conversions;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtAprilTagCmd;
import frc.robot.commands.AimShootCmd;
import frc.robot.commands.AmpShootCmd;
import frc.robot.commands.CollectCmd;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.FullFloorIntakeCmd;
import frc.robot.commands.NormalCollectCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.Autos.AutoShootCmd;
import frc.robot.commands.Autos.AutoShootCollectForwardCmd;
import frc.robot.commands.Autos.AutoShootCollectForwardShotCmd;
import frc.robot.commands.Autos.AutoShootMoveCmd;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


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

  private final XboxController xboxControllerDrive = new XboxController(OperatorConstants.kXboxDrivePort);
  private final XboxController xboxControllerButton = new XboxController(OperatorConstants.kXboxButtonPort);
  private final Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  private final Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(this.xboxControllerDrive);


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Autonomous chooser
    autoChooser.setDefaultOption("Empty", null);
    autoChooser.addOption("Shoot", new AutoShootCmd(this.shooterSubsystem, this.intakeSubsystem));
    autoChooser.addOption("Shoot Move", new AutoShootMoveCmd(this.shooterSubsystem, this.intakeSubsystem, this.chassisSubsystem));
    autoChooser.addOption("Shoot Collect Forward", new AutoShootCollectForwardCmd(this.shooterSubsystem, this.intakeSubsystem,this.chassisSubsystem));
    autoChooser.addOption("Shoot Collect Forward Shoot",new AutoShootCollectForwardShotCmd(this.shooterSubsystem, this.intakeSubsystem, this.chassisSubsystem));

    SmartDashboard.putData("Auto Chooser",autoChooser);


    configureBindings();

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
    }
    else if (leftJoystick.isConnected() && rightJoystick.isConnected()) {
      this.chassisSubsystem.setDefaultCommand(
      new DefaultTeleopCommand(this.chassisSubsystem,
      () -> -leftJoystick.getY(),
      () -> -leftJoystick.getX(),
      () -> -rightJoystick.getX()));
    }

    if (xboxControllerButton.isConnected()) {
      configureXboxBinding(OperatorConstants.kXboxButtonPort);
    }
  }

  
  private void configureXboxBinding(int port) {
    CommandXboxController cmdXboxController = new CommandXboxController(port);
    // Applies zero heading method instant command to start button trigger
    cmdXboxController.start().onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));

    // RIGHT TRIGGER
    cmdXboxController.rightTrigger().whileTrue(new NormalCollectCmd(this.intakeSubsystem));
    cmdXboxController.rightTrigger().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(INTAKE_STATE.restState);
      this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations);
      LEDController.setActionState(LEDController.ActionStates.DEFAULT);
    }));

    // A TRIGGER
    cmdXboxController.a().whileTrue(new FullFloorIntakeCmd(this.chassisSubsystem, this.intakeSubsystem, () -> cmdXboxController.back().getAsBoolean()));
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
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
