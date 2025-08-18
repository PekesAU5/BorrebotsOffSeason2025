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
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.ClimbSubsytem;
import frc.robot.Subsystems.DriveSubsystem;
// import frc.robot.Subsystems.ElevatorSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

/** Add your docs here. */
public class RobotContainer {

private final SendableChooser<Command> autoChooser; 

 // The robot's subsystems
 private final DriveSubsystem m_robotDrive = new DriveSubsystem();

private final Limelight limelight = new Limelight(m_robotDrive);


// The driver's controller
 XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
 XboxController m_driverController1 = new XboxController(OIConstants.kDriverController1Port);

 /**
  * The container for the robot. Contains subsystems, OI devices, and commands.
  */
  
 public RobotContainer() {

 DriverStation.silenceJoystickConnectionWarning(true);
    
    // Build an auto chooser. This will use Commands.none() as the default option.

   autoChooser = AutoBuilder.buildAutoChooser();
  
   HttpCamera limelightFeed = new HttpCamera(limelight.LL, "http://10.47.82.11:5800");
   
    CameraServer.startAutomaticCapture(limelightFeed);
       SmartDashboard.putData("AutoChooser", autoChooser);


shuffleboardData(limelightFeed);


   SmartDashboard.putNumber("Tx Value", limelight.getTx());
    
    // Configure the button bindings
   configureButtonBindings();

   // Configure default commands
   m_robotDrive.setDefaultCommand(
       // The left stick controls translation of the robot.
       // Turning is controlled by the X axis of the right stick.
       new RunCommand(
           () -> m_robotDrive.drive(
               -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
               -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
               -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
               DriveConstants.fieldRelative, true),
           m_robotDrive));

        //    m_climb.setDefaultCommand(new RunCommand(() -> m_climb.Climb(m_driverController1.getRightY()), m_climb));
        
        //    m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.stop(), m_elevator));
  
        //    m_arm.setDefaultCommand(new RunCommand(()-> m_arm.stop(), m_arm));
  
        //    m_intake.setDefaultCommand(new RunCommand(()-> m_intake.stop(), m_intake));
        //  m_climb.setDefaultCommand(new RunCommand(() -> m_climb.StopClimb(), m_climb));
          
       
 }

 /**
  * Use this method to define your button->command mappings. Buttons can be
  * created by
  * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
  * subclasses ({@link
  * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
  * passing it to a
  * {@link JoystickButton}.
  */
 private void configureButtonBindings() {



    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(new InstantCommand(() ->m_robotDrive.changeSpeed(), m_robotDrive));

  

    

  

   
   
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.05).whileTrue(limelight.allignAllReef(true));
    
  new Trigger(()-> m_driverController.getLeftTriggerAxis() >0.05).whileTrue(limelight.allignAllReef(false));


    // new Trigger(() -> m_driverController.getRightTriggerAxis() > 0).whileTrue(limelight.allignAllWithJoyStickAndGyro(6.0, 3.0, m_driverController));
    
    // new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0).whileTrue(limelight.allignAllWithJoyStickAndGyro(-3.0, 3.0, m_driverController));

    
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .onTrue(new InstantCommand(()-> m_robotDrive.changeDrivingMode(), m_robotDrive));


  
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .onTrue(new InstantCommand(()-> m_robotDrive.calibrate(), m_robotDrive));

    // new Trigger(()-> m_driverController1.getRightTriggerAxis() > 0.2).whileTrue(new RunCommand(()->m_intake.AlgaeIntake()));

 

    
    
   
    
 }

 /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
 public Command getAutonomousCommand() {





 return autoChooser.getSelected();
}

/*public Command getAutoWithLimeLight() {
    return Commands.sequence(autoChooser.getSelected(),
                                limelight.allignXAxis(10));
}*/







    public void shuffleboardData(HttpCamera limelightFeed) {
        
        ShuffleboardTab  tab = Shuffleboard.getTab("RobotData");
        
        // tab.add(autoChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        SmartDashboard.putData(autoChooser);
        tab.addInteger("Id", () -> limelight.getId()).withWidget(BuiltInWidgets.kTextView);
        tab.addBoolean("Id Detected", ()-> limelight.hasTarget()).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addDouble("Navx limited heading", () -> m_robotDrive.getLimitedGyroYaw());
        tab.addBoolean("Slow Mode", ()-> DriveConstants.kSlowMode).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addBoolean("FieldRelative", ()-> DriveConstants.fieldRelative).withWidget(BuiltInWidgets.kBooleanBox);
        tab.addDouble("Tx",()-> limelight.getTx());
        tab.addDouble("Ty",()-> limelight.getTy());
        tab.addDouble("Tz",()-> limelight.getRy());
        tab.addBoolean("IsAlligned", ()-> limelight.isAlligned());
        


    }
    }



