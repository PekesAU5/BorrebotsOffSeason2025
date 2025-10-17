// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.climbConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.Climber;
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.ClimbSubsytem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.IntakeSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
// import frc.robot.Subsystems.ElevatorSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
// import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser; 
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    // private final Limelight limelight = new Limelight(m_robotDrive);
    private final Elevator elevator = new Elevator();
    private final Climber m_climber = new Climber();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(arm);
    // private final PositionSystem positionSystem = new PositionSystem(elevator, arm);
    
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverController1Port);
    CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverController1Port);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);
   
        // NamedCommands.registerCommand("AutoAllignRight", limelight.allignAllAxisLeft());

        NamedCommands.registerCommand("L3", L3Position());
        NamedCommands.registerCommand("L2", L2Position());
        NamedCommands.registerCommand("L1", L1Position());
        NamedCommands.registerCommand("CoralStation", coralStationPosition());
        NamedCommands.registerCommand("Outtake", intake.coralOuttake());
        NamedCommands.registerCommand("Intake", intake.coralIntake());

        autoChooser = AutoBuilder.buildAutoChooser();
       
        
        shuffleboardData();

        // SmartDashboard.putNumber("Tx Value", limelight.getTx());
      CameraServer.startAutomaticCapture(0);
        
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController1.getRightX(), OIConstants.kDriveDeadband),
                    DriveConstants.fieldRelative, true),
                m_robotDrive));

        // elevator.setDefaultCommand(
        //     new RunCommand(
        //         () -> elevator.setManual(
        //             -MathUtil.applyDeadband(m_driverController1.getLeftY(), Constants.Elevator.kElevatorDeadband)
        //         ),elevator
        //     )
        // );

        // arm.setDefaultCommand( 
        //     new RunCommand(
        //     ()->arm.manualSpeed(
        //         -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kArmDeadband), m_driverController1) 
        //         , arm)
        //         );


        intake.setDefaultCommand(intake.defaultCommand());

       m_climber.setDefaultCommand(m_climber.setPower(MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)));
    
    }

    private void configureButtonBindings() {



    

  
    // m_driverController1.leftBumper().whileTrue(limelight.allignAllAxisLeft());
    // m_driverController1.rightBumper().whileTrue(limelight.allignAllAxisRight());


    m_driverController1.leftTrigger(OIConstants.kTriggerTreshold).whileTrue(intake.coralIntake());
    m_driverController1.rightTrigger(OIConstants.kTriggerTreshold).whileTrue(intake.coralOuttake());

// m_driverController1.povUp().onTrue(Commands.runOnce(()->changeAlgaeMode()));

m_driverController1.povLeft().onTrue(coralStationPosition());

if (OIConstants.algaeMode) {
  m_driverController1.b().onTrue(LowAlgaePosition());
m_driverController1.x().onTrue(HighAlgaePosition());
m_driverController1.y().onTrue(NetPosition());  
}else{

    m_driverController1.y().onTrue(L4Position());
    m_driverController1.x().onTrue(L3Position());
    m_driverController1.b().onTrue(L2Position());
    m_driverController1.a().onTrue(L1Position());
}


}

    


    

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    }

    public void shuffleboardData() {
        ShuffleboardTab  tab = Shuffleboard.getTab("RobotData");
        SmartDashboard.putData(autoChooser);
        // tab.addInteger("Id", () -> limelight.getId()).withWidget(BuiltInWidgets.kTextView);
        // tab.addBoolean("Id Detected", ()-> limelight.hasTarget()).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addDouble("Navx limited heading", () -> m_robotDrive.getLimitedGyroYaw());
        tab.addBoolean("Slow Mode", ()-> DriveConstants.kSlowMode).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addDouble("CurrentMaxSpeed", ()->DriveConstants.kMaxSpeedMetersPerSecond);
        tab.addBoolean("FieldRelative", ()-> DriveConstants.fieldRelative).withWidget(BuiltInWidgets.kBooleanBox);
        // tab.addDouble("Tx",()-> limelight.getTx());
        // tab.addDouble("Ty",()-> limelight.getTy());
        // tab.addDouble("Tz",()-> limelight.getRy());
        // tab.addBoolean("IsAlligned", ()-> limelight.isAlligned());
        tab.addDouble("ArmAngle", ()->arm.getAngle());
        tab.addBoolean("AlgaeMode", ()->OIConstants.algaeMode).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addDouble("MotorVoltage", ()->arm.getMotorVoltage());
    }



    public Command L4Position(){
        return arm.setArmAngle(armConstants.L4).andThen(elevator.moveToL4Command());
      }
      
      public Command L3Position(){
        return elevator.moveToL3Command().alongWith(arm.setArmAngle(armConstants.L3));
      }
      
      public Command L2Position(){
        return elevator.moveToL2Command().alongWith(arm.setArmAngle(armConstants.L2));
      }

      public Command L1Position(){
        return elevator.moveToL1Command().alongWith(arm.setArmAngle(armConstants.L1));
      }

      
      public Command LowAlgaePosition(){
        return elevator.moveToLowAlgaeCommand().alongWith(arm.setArmAngle(armConstants.REEFALGAE));
      }
      
      public Command HighAlgaePosition(){
        return elevator.moveToLowAlgaeCommand().alongWith(arm.setArmAngle(armConstants.REEFALGAE));
      }
      
      public Command NetPosition(){
        return elevator.movetoNetCommand().alongWith(arm.setArmAngle(armConstants.NET));
      }
      public Command RestPosition(){
        return elevator.moveToRestCommand().alongWith(arm.setArmAngle(armConstants.kStartpos));
      }
      public Command coralStationPosition(){
        return elevator.moveToCoralStation().alongWith(arm.setArmAngle(armConstants.CORALSTATION));
      }
      
      
      

     

  

  }




