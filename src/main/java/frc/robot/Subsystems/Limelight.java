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
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.limelightConstants;



/** Add your docs here. */
public class Limelight {
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


  

    public boolean isAlligned(){
        if(
            zPidController.getError() < limelightConstants.yReefSetpoint && 
            yPidController.getError() < limelightConstants.xReefSetpoint &&
            xPidController.getError() < limelightConstants.rotReefSetpoint){
            return true;
         }else{
                return false;
         }
    }
    public Command allignAllAxisRight() {

        return Commands.run(() -> {
            if (hasTarget()) {
                xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
                // DriveConstants.kSlowMode = false;
                 double xVelocity = -xPidController.calculate(getTx());
                 double yVelocity = yPidController.calculate(getTy());
                double zVelocity = -zPidController.calculate(getRy());

    driveSubsystem.drive(yVelocity,xVelocity , zVelocity, false, true);}
       
    }, driveSubsystem);}

    public Command allignAllAxisLeft() {

        return Commands.run(() -> {
            if (hasTarget()) {
                xPidController.setSetpoint(limelightConstants.xLeftReefSetpoint);
                // DriveConstants.kSlowMode = false;
                 double xVelocity = -xPidController.calculate(getTx());
                 double yVelocity = yPidController.calculate(getTy());
                double zVelocity = -zPidController.calculate(getRy());

    driveSubsystem.drive(yVelocity,xVelocity , zVelocity, false, true);}
       
    }, driveSubsystem);
        
    }



    

//     public Command AutoReefAllignAllAxis(int IdPriority, boolean isRightReef){
      
//         return Commands.run(() ->{
            
//             LimelightHelpers.setPipelineIndex(LL, 0);
//             LimelightHelpers.setPriorityTagID(LL, IdPriority);
            
//             allignAllReef(isRightReef);

//         }, driveSubsystem);

// }



    
}
