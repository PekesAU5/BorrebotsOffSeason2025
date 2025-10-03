// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystems;



// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.armConstants;


// public class ArmSubsystem extends SubsystemBase {

//   private SparkMax m_armMotor = new SparkMax(armConstants.karmMotorId, MotorType.kBrushless);

//   private RelativeEncoder m_Encoder;


//   private SparkMaxConfig m_Config;

//   private ProfiledPIDController m_Controller;

//   /** Creates a new ArmSubsystem. */
//   public ArmSubsystem() {
//     m_Config = new SparkMaxConfig();

//     m_Encoder = m_armMotor.getEncoder();

//     m_Config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(false);
//     m_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).velocityFF(armConstants.armVelocityFF);
//     m_Config.encoder.positionConversionFactor(armConstants.kAngleFactor).velocityConversionFactor(armConstants.kAngleFactor / 60);
//     m_Config.softLimit.forwardSoftLimit(armConstants.forwardAngleLimit).reverseSoftLimit(armConstants.reverseAngleLimit)
//     .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

  
  
    

//     m_armMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

//     m_Controller = new ProfiledPIDController(
//                                               armConstants.kP, armConstants.kI, armConstants.kD,
//                                                new Constraints(armConstants.kMaxAngularVelocity, armConstants.kMaxAngularAccelearion));

//     m_Encoder.setPosition(0);

//   }




//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("armAngle", getAngle());
//     SmartDashboard.putNumber("armVelocity", m_Encoder.getVelocity());
//     SmartDashboard.putNumber("armVoltage", m_armMotor.getBusVoltage());
//     SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
//   }

//   public double getAngle(){
//     return m_Encoder.getPosition();
//   }


//   /**
//    * 
//    * @param degrees the setpoint
//    * @return if the arm is near the setpoint
//    */
//   public boolean aroundAllowableAngle(double degrees){
//     return MathUtil.isNear(degrees, getAngle(), armConstants.kAllowableError);
//   }


//   /**
//    * 
//    * @param angle the setpoint to reach
//    * @return the command
//    */
//   public Command reachSetpoint(double angle){

    
//     return Commands.run(()->{

//       m_armMotor.setVoltage(MathUtil.clamp(m_Controller.calculate(getAngle(), angle), -8, 8));
//     }, this);
 
//   }

//   /**
//    * 
//    * @param angle The desired angle to reach
//    * @return the command
//    */
//   public Command setArmAngle(double angle){

//     return reachSetpoint(angle).beforeStarting(()->{m_Controller.reset(getAngle());})
//     .until(()->aroundAllowableAngle(angle)).finallyDo(()->stop());
//   }

//   /**
//    * 
//    * @return stops the motor
//    */
//   public Command stop(){
//     return Commands.run(()->{
//       m_armMotor.set(0.0);
//     }, this);
//   }

//   public double angletoHold = 0.0;

// /**
//  * 
//  * @return holds the current angle
//  */
//   public Command angleHold(){
//     return startRun(()->{
//       angletoHold = getAngle();
//       m_Controller.reset(angletoHold);
//     }, 
//     ()->{
//       reachSetpoint(angletoHold);
//   });
//   }


//   /**
//    * 
//    * @param power the power to give the motors
//    * @return the command
//    */
//   public Command manualSpeed(double power){
//     return Commands.run(()->{
//       m_armMotor.set(power);

//   },this);
//   }
  







// }
