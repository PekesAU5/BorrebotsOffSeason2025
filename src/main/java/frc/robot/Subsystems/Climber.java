// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // WPI_VictorSPX climberMotor = new WPI_VictorSPX(7);
private SparkMax climberMotor = new SparkMax(3, MotorType.kBrushless);
private SparkMaxConfig climberConfig = new SparkMaxConfig();

private RelativeEncoder climberEncoder;

  public Climber() {

  climberEncoder = climberMotor.getEncoder();

  climberConfig.inverted(false).smartCurrentLimit(50).idleMode(IdleMode.kBrake);

  // climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

  climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  climberEncoder.setPosition(0);

  

  }
<<<<<<< Updated upstream

public Command setPower(double power){
  return Commands.run(()->{
    climberMotor.set(power);
  }, this);
}
=======
public Command Climb(){ 
  return Commands.run(() -> 
  {climberMotor.set(1.0);
  } ,this);
}


public Command stopClimb(){
  return Commands.run(() -> 
  {climberMotor.set(0.0);
  }, this);
}

  public Command unflexClimb(){
    return Commands.run(() -> {
      climberMotor.set(-1.0);
    }, this);
  }
  
>>>>>>> Stashed changes

  @Override
  public void periodic() {

  }

  public double climberRev(){
   return climberEncoder.getPosition();
  }


  
}
