// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

public class TrapSubsystem extends SubsystemBase {
  /** Creates a new TrapSubsystem. */
  private final CANSparkMax shooterMotor; //announce on a new SparkMax motor
  private final DigitalInput LimitSwitchTrap; // announce on a new limit switch
  private final DoubleSolenoid elevatorPiston; // announce on a new double solenoid

  public TrapSubsystem() {
      this.shooterMotor = new CANSparkMax(TrapConstants.kTrapShooterMotorPort,MotorType.kBrushless);// sets the motor to its port and sets it to bruchless
      this.LimitSwitchTrap = new DigitalInput(TrapConstants.kLimitSwitchPort);// sets this limit switch to its port
      this.elevatorPiston = 
          new DoubleSolenoid(PneumaticsModuleType.CTREPCM, TrapConstants.kDoubleSelenoidForward, TrapConstants.kDoubleSelenoidReverse); // sets it to our module type and its ports
  }

  private void releaseElevator(){ // method that releases the piston of the elevator
    elevatorPiston.set(DoubleSolenoid.Value.kForward);
  }

  private void shootPiece(){ // method that shoots the game piece with the speed that it has to be
    shooterMotor.set(TrapConstants.kMotorSpeed);
  }

  private void collectPiece(){ // method that collect the piece with the opposite speed of the shooting
    shooterMotor.set(TrapConstants.kMotorSpeedCollector);
  }

  private void stopMotor(){ // method that stop the motors
    shooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

