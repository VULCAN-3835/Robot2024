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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.STATE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.FullFloorIntakeCmd;
import frc.robot.commands.ShootCmd;
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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final XboxController xboxControllerDrive = new XboxController(OperatorConstants.kXboxPort);
  private final CommandXboxController cmdXboxControllerDrive = new CommandXboxController(OperatorConstants.kXboxPort);

  private final XboxController xboxControllerButton = new XboxController(1);
  private final CommandXboxController cmdXboxControllerButton = new CommandXboxController(1);

  // private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(this.chassisSubsystem,
    () -> -xboxControllerDrive.getLeftY(),
    () -> -xboxControllerDrive.getLeftX(),
    () -> -xboxControllerDrive.getRightX()));

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
    // registerCommands();
    // shuffleboardPaths();
  }
  // private void registerCommands() {
  //   // Register named commands
  //   NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
  //   NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
  //   NamedCommands.registerCommand("print hello", Commands.print("Hello"));
  // }

  // private void shuffleboardPaths() {
  //   SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

  //   // Add a button to run pathfinding commands to SmartDashboard
  //   SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
  //     new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0)), 
  //     new PathConstraints(
  //       4.0, 4.0, 
  //       Units.degreesToRadians(360), Units.degreesToRadians(540)
  //     ), 
  //     0, 
  //     2.0
  //   ));
  //   SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
  //     new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
  //     new PathConstraints(
  //       2.0, 2.0, 
  //       Units.degreesToRadians(360), Units.degreesToRadians(540)
  //     ), 
  //     0, 
  //     0
  //   ));

  //   // Add a button to SmartDashboard that will create and follow an on-the-fly path
  //   // This example will simply move the robot 2m in the +X field direction
  //   SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
  //     Pose2d currentPose = this.chassisSubsystem.getPose();
      
  //     // The rotation component in these poses represents the direction of travel
  //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
  //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

  //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
  //     PathPlannerPath path = new PathPlannerPath(
  //       bezierPoints, 
  //       new PathConstraints(
  //         2.0, 2.0, 
  //         Units.degreesToRadians(360), Units.degreesToRadians(540)
  //       ),  
  //       new GoalEndState(0.0, currentPose.getRotation())
  //     );

  //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
  //     path.preventFlipping = true;

  //     AutoBuilder.followPath(path).schedule();
  //   }));
  // }

  private void configureBindings() {
    // Initilizing a start button trigger
    Trigger rightBumperTrigger = new Trigger(() -> this.xboxControllerDrive.getRightBumper());
    Trigger leftBumperTrigger = new Trigger(() -> this.xboxControllerDrive.getLeftBumper());

    // Applies zero heading method instant command to start button trigger
    cmdXboxControllerDrive.start().onTrue(new InstantCommand(() -> this.chassisSubsystem.zeroHeading()));

    // Applyies intake to intake motor on right bumper
    rightBumperTrigger.whileTrue(new InstantCommand(() -> {
      this.intakeSubsystem.setRotationPosition(0.41);
      }));
    rightBumperTrigger.toggleOnFalse(new InstantCommand(() -> { 
       this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations); 
      }));

    // Applies output to shooter motor on right trigger
    cmdXboxControllerDrive.rightTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.setShooterSpeed(ShooterConstants.kShootPower)));
    cmdXboxControllerDrive.rightTrigger().toggleOnFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));

    // Applies intake to shooter motor on left trigger
    cmdXboxControllerDrive.leftTrigger().whileTrue(new InstantCommand(() -> this.shooterSubsystem.collect()));
    cmdXboxControllerDrive.leftTrigger().toggleOnFalse(new InstantCommand(() -> this.shooterSubsystem.stopMotor()));

    // Applies output to intake motor on left bumper
    leftBumperTrigger.whileTrue(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.collectState)));
    leftBumperTrigger.toggleOnFalse(new InstantCommand(() -> this.intakeSubsystem.setMotorMode(STATE.restState)));

    cmdXboxControllerDrive.a().whileTrue(new FullFloorIntakeCmd(this.chassisSubsystem, this.intakeSubsystem, () -> xboxControllerDrive.getBackButton()));
    cmdXboxControllerDrive.a().toggleOnFalse(new InstantCommand(() -> {
       this.intakeSubsystem.setMotorMode(STATE.restState);
       this.intakeSubsystem.setRotationPosition(IntakeConstants.kClosedRotations); 
      }));

    cmdXboxControllerDrive.y().whileTrue(new AimShootCmd(chassisSubsystem, intakeSubsystem, shooterSubsystem, () -> xboxControllerDrive.getBackButton()));
    cmdXboxControllerDrive.y().toggleOnFalse(new InstantCommand(() -> {
       this.intakeSubsystem.setMotorMode(STATE.restState);
       this.shooterSubsystem.setShooterSpeed(0); 
      }));


    cmdXboxControllerDrive.x().whileTrue(new InstantCommand(() -> {
       this.intakeSubsystem.setMotorMode(STATE.collectState);
       this.shooterSubsystem.setShooterSpeed(ShooterConstants.kCollectPower); 
      }));
    cmdXboxControllerDrive.x().toggleOnFalse(new InstantCommand(() -> {
      this.intakeSubsystem.setMotorMode(STATE.restState);
      this.shooterSubsystem.setShooterSpeed(0); 
    }));
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
