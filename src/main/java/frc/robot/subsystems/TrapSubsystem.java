// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.trapConstants;

public class TrapSubsystem extends SubsystemBase {
  /** Creates a new TrapSubsystem. */
  Spark shooterMotor = new Spark(trapConstants.kTrapMotorPort);
  DigitalInput LimitSwitch = new DigitalInput(trapConstants.kLimitSwitchPort);
  DigitalInput pieceSwitch = new DigitalInput(trapConstants.kPieceLimitSwitch);
  public TrapSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
