// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  WPI_VictorSPX climberMotor = new WPI_VictorSPX(7);

  public Climber() {
    climberMotor.setInverted(false);
    climberMotor.setNeutralMode(NeutralMode.Brake);



  }
public Command frontClimb(){ 
  return Commands.run(() -> {climberMotor.set(0.7);} ,this);}

public Command stopClimb(){
  return Commands.run(() -> {climberMotor.set(0.0);}, this);}

  public Command unflexClimb(){
    return Commands.run(() -> {climberMotor.set(-0.7);}, this);}
  

  @Override
  public void periodic() {
    
  }
}
