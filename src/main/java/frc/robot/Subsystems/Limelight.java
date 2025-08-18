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
        yPidController = new PIDController(0.02, 0.0, 0.0);
        xPidController = new PIDController(0.0125, 0.0, 0.0);

       
        // xPidController.setTolerance(limelightConstants.xReefTolerance);

        yPidController.setSetpoint(limelightConstants.yReefSetpoint);
        // yPidController.setTolerance(limelightConstants.yReefTolerance);

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
        return NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("tx").getDouble(0);
       
      
    }

    
    public double getTy(){
    
        return NetworkTableInstance.getDefault().getTable(LL).getEntry("ty").getDouble(0);
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
        if(zPidController.getError() < limelightConstants.rotReefTolerance && 
         yPidController.getError() < limelightConstants.yReefTolerance &&
         xPidController.getError() < limelightConstants.xReefTolerance){
            return true;
         }else{
                return false;
         }
    }

    public Command allignAllAxis() {

        return Commands.run(() -> {
            // limelightConstants.isRightReef = isRightReef;
            //   xPidController.setSetpoint(limelightConstants.xReefSetpoint);
            if (hasTarget()) {
                double xVelocity = -xPidController.calculate(getTx())*0.7 ;
                double yVelocity = -yPidController.calculate(getTy()) * 0.5;
                // double zVelocity = -zPidController.calculate(getTx(), xSetPoint)*0.75 - (controller.getRightX()*0.25);
                double zVelocity = -zPidController.calculate(getRy())*0.7;
                    
// if(!isAlligned()){
    // driveSubsystem.drive(0, 0, 0, false, true);
// }
    driveSubsystem.drive(yVelocity, xVelocity, zVelocity, false, true);}
             

            
    }, driveSubsystem);
        
    }

    public Command allignAllWithJoyStick(double xSetPoint, double ySetPoint, XboxController controller) {
        return Commands.run(() -> {
            double xVelocity = -xPidController.calculate(getTx(), xSetPoint) * 0.5;
            double yVelocity = -yPidController.calculate(getTy(), ySetPoint) * 0.5;
            double zVelocity = (-zPidController.calculate(getTx(), xSetPoint)) - controller.getRightX() * 0.25;

            driveSubsystem.drive(yVelocity, xVelocity, zVelocity, false, true);
        }, driveSubsystem);
    }

    public Command allignAllWithJoyStickAndId(XboxController controller) {
        return Commands.run(() -> {
            if (hasTarget()) {
                double xVelocity = -xPidController.calculate(getTx(), 3) * 0.5;
                double yVelocity = -yPidController.calculate(getTy(), 3) * 0.5;
                double zVelocity = -zPidController.calculate(getRy())*0.75 - (controller.getRightX()*0.25);
                // double zVelocity = zPidController.calculate(driveSubsystem.getLimitedGyroYaw(), ReefAngles.get(18))
                    // + (controller.getRightX() * 0.25);

                driveSubsystem.drive(yVelocity, xVelocity, zVelocity, false, true);

             
           
            }
        }, driveSubsystem);
    }

    public Command allignAllReef(boolean isRightReef) {
    
    

    if(isRightReef){
       xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
    }else{
        xPidController.setSetpoint(limelightConstants.xLeftReefSetpoint);
    }
     
        return allignAllAxis().until(()->isAlligned()).finallyDo(()->stopCommand());
       
    }


    

    public Command AutoReefAllignAllAxis(int IdPriority, boolean isRightReef){
      
        return Commands.run(() ->{
            
            LimelightHelpers.setPipelineIndex(LL, 0);
/*if(DriverStation.getAlliance().get() ==  DriverStation.Alliance.Red){
    Id = reefside;
}else{
    switch (reefside) {
        case 10: Id = 21; break;
    case 9: Id = 22; break;
    case 8: Id = 17; break;
    case 7: Id = 18; break;
    case 6: Id = 19; break;
    case 11: Id = 20; break;
    
    }
}*/
LimelightHelpers.setPriorityTagID(LL, IdPriority);
            
            allignAllReef(isRightReef);

        }, driveSubsystem);

}



    
}
