// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//constants
//public static class ClimberSubsystemConstants{
//  public static final int motor1Port=10;
//  public static final int motor2Port=11;
//  public static final int limSwitchRightPort=0;
//  public static final int limSwitchLeftPort=1;
//  public static final double maxMotorPower=0.7;
//  public static final double kpForLift TODO
//  public static final double ksForLift TODO
//  public static final double motorPowerPrep TODO
//  public static final int ticsForRotation TODO
//  public static final double lengthForRotation =125.6637061435917;
//}




public class ClimberSubsystem extends SubsystemBase {
  private TalonFX climberMotorRight;
  private TalonFX climberMotorLeft;
  private DigitalInput limitSwitchRight;
  private DigitalInput limitSwitchLeft;
  private PIDController positionController;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
      this.climberMotorRight=new TalonFX(Constants.ClimberSubsystemConstants.motor1Port);
      this.climberMotorLeft=new TalonFX(Constants.ClimberSubsystemConstants.motor2Port);
      this.limitSwitchRight=new DigitalInput(Constants.ClimberSubsystemConstants.limSwitchRightPort);
      this.limitSwitchRight=new DigitalInput(Constants.ClimberSubsystemConstants.limSwitchLeftPort);
      this.positionController=new PIDController(0.5, 0, 0);

  }

  public boolean getRightLimitSwitch(){
    return this.limitSwitchRight.get();
  }
   public boolean getLeftLimitSwitch(){
    return this.limitSwitchLeft.get();
  }
  public boolean prepForClimb(){
    while(!getLeftLimitSwitch()&&!getRightLimitSwitch()){
        if(!getLeftLimitSwitch()){
          this.climberMotorLeft.set(Constants.ClimberSubsystemConstants.motorPowerPrep);
        }
        if(!getRightLimitSwitch()){
          this.climberMotorRight.set(Constants.ClimberSubsystemConstants.motorPowerPrep);
        }
    }
    this.climberMotorLeft.setPosition(0);
    this.climberMotorRight.setPosition(0);
    //TODO send message to dash board saying climb ready
    return true;
  }
  // public boolean setClimberPosition(double postion){
  //   double leftErorr;
  //   double rightError;
  //   double neededTics;
  //   if(prepForClimb()){
  //       neededTics=(postion/Constants.ClimberSubsystemConstants.lengthForRotation)
  //       *Constants.ClimberSubsystemConstants.ticsForRotation;
  //       while(this.climberMotorLeft.getPosition()!=neededTics&&this.climberMotorRight.getPosition()!=neededTics){
  //         leftErorr=neededTics-this.climberMotorLeft.getPosition();
  //         rightError=neededTics-this.climberMotorRight.getPosition();
  //         this.climberMotorRight.set(leftErorr*Constants.ClimberSubsystem.kpForLift+Constants.ClimberSubsystem.ksForLift);
  //         this.climberMotorLeft.set(leftErorr*Constants.ClimberSubsystem.kpForLift+Constants.ClimberSubsystem.ksForLift);
  //       }
  //   }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
