// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private ArmSubsystem arm;
  private SparkMax intakeMotor;
//   private DigitalInput sensor = new DigitalInput(1);
  private SparkMaxConfig config;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(ArmSubsystem arm) {
  intakeMotor = new SparkMax(2, MotorType.kBrushless);
  config = new SparkMaxConfig();
  config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true);
  config.closedLoop.outputRange(-1, 1);
    this.arm = arm;

    }

  

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public Boolean objectDetected(){
//   return sensor.get();
//   }

  public Command defaultCommand(){
    if(OIConstants.algaeMode){
      return Commands.run(()->{ intakeMotor.set(0.4);}, this);
      }else{
        return Commands.run(()->{intakeMotor.set(0.0);},this);
      }
    }
  
  public Command coralIntake(){
    return Commands.run(()->{
      if(arm.getAngle() <60){
        intakeMotor.set(-0.6);
      }else{
        intakeMotor.set(0.6);;
      }}, this);
  }

  public Command algaeIntake(){
    return Commands.run(()->{
      intakeMotor.set(1.0);
    }, this);
// .until(()-> objectDetected()).andThen(defaultCommand());
  }

//   public Command corall2Outtake(){
//     return Commands.run(()->{intakeMotor.set(-0.4);}, this);
//   }
  public Command coralOuttake(){
    return Commands.run(()->{
      if(arm.getAngle() <60){
        intakeMotor.set(0.8);
      }else{
        intakeMotor.set(-0.8);;
      }}, this);
   
  }

  public Command algaenetOuttake(){
    return Commands.run(()->{intakeMotor.set(-1.0);}, this);
  }
  

//   public Command algaeProcessorOuttake(){
//     return Commands.run(()->{intakeMotor.set(-0.8);}, this);
//   }
}
