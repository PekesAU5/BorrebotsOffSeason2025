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
import frc.robot.Constants.climbConstants;
import frc.robot.Subsystems.Climber;
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.ClimbSubsytem;
import frc.robot.Subsystems.DriveSubsystem;
// import frc.robot.Subsystems.ElevatorSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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

private final Climber m_climber = new Climber();


// The driver's controller
 CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
 CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverController1Port);

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
       


shuffleboardData();

    
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

          
       m_climber.setDefaultCommand(m_climber.setPower(climbConstants.stopClimb));

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



m_driverController.leftBumper().onTrue(m_robotDrive.changeSpeed());
    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).onTrue(m_robotDrive.changeSpeed());
m_driverController.povUp().whileTrue(m_climber.setPower(climbConstants.climbPower));
m_driverController.povDown().whileTrue(m_climber.setPower(climbConstants.unflexPower));

  

    
// m_driverController.rightTrigger(OIConstants.kTriggerTreshold)

   
   
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.05).whileTrue(limelight.allignAllAxisRight());
    
  new Trigger(()-> m_driverController.getLeftTriggerAxis() >0.05).whileTrue(limelight.allignAllAxisLeft());


    m_driverController.povLeft().onTrue(m_robotDrive.changeDrivingMode());

    m_driverController.povRight().onTrue(m_robotDrive.calibrate());

  

 

    
    
   
    
 }

 /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
 public Command getAutonomousCommand() {
 return autoChooser.getSelected();
}


    public void shuffleboardData() {
        
        
        
        SmartDashboard.putData(autoChooser);
        SmartDashboard.putNumber("Id", limelight.getId());
        SmartDashboard.putBoolean("Id Detected", limelight.hasTarget());
        SmartDashboard.putNumber("NavX Angle", m_robotDrive.getLimitedGyroYaw());
        SmartDashboard.putBoolean("Slow Mode", DriveConstants.kSlowMode);
        SmartDashboard.putBoolean("FieldRelative", DriveConstants.fieldRelative);
        SmartDashboard.putNumber("Tx", limelight.getTx());
        SmartDashboard.putNumber("Ty", limelight.getTy());
        SmartDashboard.putNumber("Tz", limelight.getRy());
        SmartDashboard.putBoolean("IsAlligned", limelight.isAlligned());
       


        


    }
    }



