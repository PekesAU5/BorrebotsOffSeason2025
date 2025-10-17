// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.armConstants;


public class ArmSubsystem extends SubsystemBase {

  public enum State{
    MOVING,
    IDLE,
    MANUAL;

  }

  private SparkMax m_armMotor;

  private RelativeEncoder m_Encoder;

  private State state = State.IDLE;
  private SparkMaxConfig m_Config;

  private ProfiledPIDController m_Controller;

  private double target = armConstants.kStartpos;

  private double voltagesetting = 0.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor = new SparkMax(armConstants.karmMotorId, MotorType.kBrushless);
    m_Config = new SparkMaxConfig();

    m_Encoder = m_armMotor.getEncoder();

    m_Config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(false);
    m_Config.encoder.positionConversionFactor(armConstants.kAngleFactor);
    m_Config.softLimit.forwardSoftLimit(armConstants.forwardAngleLimit).reverseSoftLimit(armConstants.reverseAngleLimit)
    .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    
  
  
    

    m_armMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    m_Controller = new ProfiledPIDController(
                                              armConstants.kP, armConstants.kI, armConstants.kD,
                                               new Constraints(armConstants.kMaxAngularVelocity, armConstants.kMaxAngularAccelearion));

    m_Encoder.setPosition(0);
    m_Controller.setTolerance(armConstants.kAllowableErrorpercent);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("armAngle", getAngle());
    SmartDashboard.putNumber("armVelocity", m_Encoder.getVelocity());
    SmartDashboard.putNumber("MotorVoltage", voltagesetting);
  }







  public double getAngle(){
    return m_Encoder.getPosition();
  }
  

public boolean nearMax(){
  return MathUtil.isNear(armConstants.forwardAngleLimit, getAngle(),1.0);
}

public boolean nearMin(){
  return MathUtil.isNear(armConstants.reverseAngleLimit, getAngle(), 1.0);
}




  public Command reachSetpoint(double angle){
   return Commands.run(()->{
    m_armMotor.setVoltage(
      MathUtil.clamp(
        m_Controller.calculate(getAngle(), angle)
        , -10, 10));

   }, this);
  }



  public void manualSpeed(double speed, CommandXboxController controller){

  if(controller.povLeft().getAsBoolean()){voltagesetting += 0.1;}
  if(controller.povRight().getAsBoolean()){voltagesetting -= 0.1;}
    if(nearMax() && speed > 0) speed = 0;
    if(nearMin() && speed < 0) speed = 0;

    m_armMotor.set(0.5*speed + voltagesetting);
  }
  public Command pivottoHomeCommand(){
    return Commands.runOnce(()-> reachSetpoint(armConstants.kStartpos), this)
    .andThen(Commands.waitUntil(()->m_Controller.atGoal()));
  }



  

  public Command pivottoL4Command(){
    return setArmAngle(armConstants.L4);
  }

  public Command movetoProcessorCommand(){
    return Commands.runOnce(()-> setArmAngle(armConstants.PROCESSOR), this)
    .andThen(Commands.waitUntil(()->m_Controller.atGoal()));
  }

  public Command pivottoReefAlgaeCommand(){
    return Commands.runOnce(()-> setArmAngle(armConstants.REEFALGAE), this)
    .andThen(Commands.waitUntil(()->m_Controller.atGoal()));
  }

  public Command pivottoNetCommand(){
    return Commands.runOnce(()-> setArmAngle(armConstants.NET), this)
    .andThen(Commands.waitUntil(()->m_Controller.atGoal()));
  }

  public double getMotorVoltage(){
    return voltagesetting;
  }





  /**
   * 
   * @param angle The desired angle to reach
   * @return the command
   */
  public Command setArmAngle(double angle){

    return reachSetpoint(angle).beforeStarting(()->m_Controller.reset(getAngle()))
    .until(()->m_Controller.atGoal());
  }

  /**
   * 
   * @return stops the motor
   */
  public Command stop(){
    return Commands.run(()->{
      m_armMotor.set(0.0);
    }, this);
  }

  public double angletoHold = 0.0;

/**
 * 
 * @return holds the current angle
 */
  public Command angleHold(double angle){
    
      angletoHold = angle;
      m_Controller.reset(getAngle());

    
      return reachSetpoint(angletoHold).onlyIf(()-> !m_Controller.atGoal());
  
  

  }

  








    
  // public void runManual(double output){
  //   state = State.MANUAL;
  //   output = Math.max(armConstants.kMinVoltage, Math.min(armConstants.kMaxVoltage, output));

  //   // Soft limits
  //   if (getAngle() <= armConstants.kMinAngle && output < 0) output = 0;
  //   if (getAngle() >= armConstants.kMaxAngle && output > 0) output = 0;

  //   if(output < OIConstants.kArmDeadband) state = State.IDLE;
  //   m_armMotor.set(output*0.5);

  // }

  

  
   

  //   /**
  //  * 
  //  * @param angle the setpoint to reach
  //  */
  // public void reachSetpoint(double angle){
    
  //   target = Math.max(Constants.Elevator.MINheight,
  //   Math.min(Constants.Elevator.MAXheight, angle));
  //   m_Controller.reset(getAngle());
  //   state = State.MOVING;
  //   // return Commands.run(()->{

  //   //   m_armMotor.setVoltage(MathUtil.clamp(m_Controller.calculate(getAngle(), angle), -8, 8));
  //   // }, this);
 
  // }

  // @Override
  // public void periodic() {
  //   SmartDashboard.putNumber("armAngle", getAngle());
  //   SmartDashboard.putNumber("armVelocity", m_Encoder.getVelocity());
  //   SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
  //   SmartDashboard.putString("ArmState", state.name());
  //   SmartDashboard.putNumber("target", target);
  //   m_Controller.setP(SmartDashboard.getNumber("armP", armConstants.kP));
  //   m_Controller.setD(SmartDashboard.getNumber("armD", armConstants.kD));

  //   switch (state) {
  //     case MOVING:
  //       double output = m_Controller.calculate(getAngle(), target);

  //       output = Math.max(Constants.Elevator.MIN_OUTPUT, Math.min(Constants.Elevator.MAX_OUTPUT, output));
  //       m_armMotor.setVoltage(MathUtil.clamp(output, -8, 8));
        
        
   
  //       if(m_Controller.atGoal()) state = State.IDLE;
   
  //       break;



  //     case IDLE:
  //     m_armMotor.set(0);
  //     break;

  //     case MANUAL:
  //       break;

  //   }
  // }

  

 




  }