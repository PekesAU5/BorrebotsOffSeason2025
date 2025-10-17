// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.math.MathUsageId;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.armConstants;
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.Elevator;
// import frc.robot.Subsystems.IntakeSubsystem;

// /** Add your docs here. */
// public class ScoringSystem {

//     private Elevator elevator = new Elevator();
//     private ArmSubsystem arm = new ArmSubsystem();
//     private IntakeSubsystem intake = new IntakeSubsystem();


//     public ScoringSystem(Elevator elevator, ArmSubsystem arm, IntakeSubsystem intake){
//         this.intake = intake;
//         this.elevator = elevator;
//         this.arm = arm;

//     }


//     public boolean isNearPosition(double armAngle, double elevatorHeight){
//         if(MathUtil.isNear(armAngle, arm.getAngle(), 1.0) 
//         && MathUtil.isNear(elevatorHeight, elevator.getPositionMeters(), Constants.Elevator.kAllowableErrorHeight)){
//             return true;
//         }else{
//             return false;
//         }
//     }


//     public Command scoreCoral(){
//         if(isNearPosition(armConstants.L2, Constants.Elevator.L2)){
//             return intake.corall2Outtake();}
//         else{
//             return intake.coralOuttake();
//         }
//         }
    
//     public Command scoreAlgae(){
//         if(isNearPosition(armConstants.PROCESSOR, Constants.Elevator.LowPOS)){
//             return intake.algaeProcessorOuttake();
//         }else{
//             return intake.algaenetOuttake();
//         }
//     }


//     public Command ScoreElements(){
//         if(Constants.algaeMode){
//             return scoreAlgae();
//         }else{
//             return scoreCoral();
//         }
//     }
// }
