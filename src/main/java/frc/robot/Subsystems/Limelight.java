// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.limelightConstants;



/** Add your docs here. */
public class Limelight extends SubsystemBase{
    public String LL = "limelight-ll" ;
    
    DriveSubsystem driveSubsystem;
    PIDController zPidController;
    PIDController yPidController;
    PIDController xPidController;
    
    // double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL); // [0] x, [2] y, [4], z/rot
    Map<Integer, Double> ReefAngles = new HashMap<>();





    public Limelight(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        zPidController = new PIDController(0.0125, 0.0, 0.0);
        yPidController = new PIDController(0.035, 0.0, 0.0);
        xPidController = new PIDController(0.035, 0.0, 0.0);

        // xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
        xPidController.setTolerance(limelightConstants.xReefTolerance);

        yPidController.setSetpoint(limelightConstants.yReefSetpoint);
        yPidController.setTolerance(limelightConstants.yReefTolerance);

        zPidController.setSetpoint(limelightConstants.rotReefSetpoint);
        // zPidController.setTolerance(limelightConstants.rotReefTolerance);
      
        zPidController.enableContinuousInput(-180.0, 180);

<<<<<<< Updated upstream
=======
        

       

        // angle values acording id
        // Left Hexagon (Blue)
        ReefAngles.put(21, 0.0);   // Given
        ReefAngles.put(22, 60.0);  // Given
        ReefAngles.put(17, 120.0); // Adjusted
        ReefAngles.put(18, 180.0); // Adjusted
        ReefAngles.put(19, 240.0); // Adjusted
        ReefAngles.put(20, 300.0); // Adjusted

        // Right Hexagon (Red)
        ReefAngles.put(10, 0.0);   // Given
        ReefAngles.put(9, 60.0);   // Given
        ReefAngles.put(8, 120.0);  // Adjusted
        ReefAngles.put(7, 180.0); // Adjusted
        ReefAngles.put(6, 240.0);  // Adjusted
        ReefAngles.put(11, 300.0);  // Adjusted
        

>>>>>>> Stashed changes
    }

   

    public double getTx(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[0]*10;
    }

    
    public double getTy(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[2]*10;
    }

    public double getRy(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[4];
    }
    public double getTa(){
        return LimelightHelpers.getTA(LL);
    }

    public double getTxnc(){
        return NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("txnc").getDouble(0);
    }
    
    public boolean hasTarget(){
        return LimelightHelpers.getTV(LL);
    }

    public int getId() {
        return (int)LimelightHelpers.getFiducialID(LL);
    }
    
    public Command stopCommand(){
        return Commands.run(()->{

            driveSubsystem.drive(0, 0, 0, false, true);
        }, driveSubsystem);
    }


  
    public void ResetPids(){
        xPidController.reset();
        yPidController.reset();
        zPidController.reset();
    }

    public boolean isAlligned(){
<<<<<<< Updated upstream
        if(
           zPidController.atSetpoint() && yPidController.atSetpoint() && xPidController.atSetpoint()){
=======
        if(xPidController.atSetpoint() &&
        yPidController.atSetpoint() && 
        zPidController.atSetpoint()
            ){
>>>>>>> Stashed changes
            return true;
         }else{
                return false;
         }
    }
    public Command allignAllAxisRight() {

        return Commands.run(() -> {
            if (hasTarget()) {
                xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
            
                 double xVelocity = -xPidController.calculate(getTx());
                 double yVelocity = yPidController.calculate(getTy());
                double zVelocity = -zPidController.calculate(getRy());

    driveSubsystem.drive(yVelocity,xVelocity , zVelocity, false, true);}
       
    }, driveSubsystem).onlyIf(()->hasTarget()).unless(()->isAlligned());}

    public Command allignAllAxisLeft() {

        return Commands.run(() -> {
            if (hasTarget()) {
                xPidController.setSetpoint(limelightConstants.xLeftReefSetpoint);
                 double xVelocity = -xPidController.calculate(getTx());
                 double yVelocity = yPidController.calculate(getTy());
                double zVelocity = -zPidController.calculate(getRy());

    driveSubsystem.drive(yVelocity,xVelocity , zVelocity, false, true);}
       
    }, driveSubsystem).onlyIf(()->hasTarget()).unless(()->isAlligned());
        
    }

    public Command allignAllAxisAlgae(){
<<<<<<< Updated upstream
        return Commands.run(() -> {
            
                xPidController.setSetpoint(limelightConstants.xAlgaeSetpoint);
                // DriveConstants.kSlowMode = false;
                 double xVelocity = -xPidController.calculate(getTx());
                 double yVelocity = yPidController.calculate(getTy());
                double zVelocity = -zPidController.calculate(getRy());

    driveSubsystem.drive(yVelocity,xVelocity , zVelocity, false, true);}
       
    , driveSubsystem).onlyIf(()->hasTarget()).unless(()->isAlligned());
    }


    // public Command AllignAllAxis(String side){
    //     switch (side) {
    //         case limelightConstants.LEFTALLIGN:
    //         return allignAllAxisLeft();
    //         case limelightConstants.RIGHTALLIGN:
    //         return allignAllAxisRight();
    //         case limelightConstants.ALGAEALLIGN:
    //         return allignAllAxisAlgae();
    //         default: return Commands.run(()->{driveSubsystem.drive(0, 0, 0, false, true);}, driveSubsystem);
    //     }
    // }

    
    // public Command allignRobot(String side){
    //     return AllignAllAxis(side).beforeStarting(()->ResetPids()).until(()-> isAlligned());
    // }

  
//     public Command AutoReefAllignAllAxis(int IdPriority, boolean isRightReef){
=======
        return Commands.run(()->{
            if (hasTarget()){
                double xVelocity = -xPidController.calculate(getTx());
                double yVelocity = yPidController.calculate(getTy());
               double zVelocity = -zPidController.calculate(getRy());
               driveSubsystem.drive(yVelocity, xVelocity , zVelocity, false, true);
            }

        }, driveSubsystem);
    }


    public Command allignAllAxis(String side){

        switch(side){
            case limelightConstants.LEFTALLIGN: return allignCommand(allignAllAxisLeft());
            case limelightConstants.RIGHTALLIGN: return  allignCommand(allignAllAxisRight());
            case limelightConstants.ALGAEALLIGN:return  allignCommand(allignAllAxisAlgae());

        default: return Commands.run(()->{
            driveSubsystem.drive(0, 0, 0, DriveConstants.fieldRelative, true);
        }, driveSubsystem);
        }
        
    }

    public Command allignCommand(Command Allignside){

        return  Allignside.until(()->isAlligned());
    }

    
    // public Command slowDownonTarget(){
    //     double lastSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
    //     boolean lastMode = DriveConstants.kSlowMode;

    //     return Commands.run(()->{
    //         if(hasTarget()){
    //             DriveConstants.kMaxSpeedMetersPerSecond = 3.6;
    //             DriveConstants.kSlowMode = true;
    //         }else{
    //             DriveConstants.kMaxSpeedMetersPerSecond = lastSpeed;
    //             DriveConstants.kSlowMode = lastMode;
    //         }
    //     });
    // }

    // public Command AutoReefAllignAllAxis(int IdPriority, boolean isRightReef){
>>>>>>> Stashed changes
      
    //     return Commands.run(() ->{
            
    //         LimelightHelpers.setPipelineIndex(LL, 0);
    //         LimelightHelpers.setPriorityTagID(LL, IdPriority);

            
            

    //     }, driveSubsystem);

// }



    
}
