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
import frc.robot.Constants.limelightConstants;
import frc.robot.Subsystems.Climber;
// import frc.robot.Subsystems.ArmSubsystem;
// import frc.robot.Subsystems.ClimbSubsytem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Elevator;
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
import com.pathplanner.lib.auto.NamedCommands;
public class RobotContainer {
    private final SendableChooser<Command> autoChooser; 
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight limelight = new Limelight(m_robotDrive);
    private final Elevator elevator = new Elevator();
    private final Climber m_climber = new Climber();

    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverController1Port);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);
   
        NamedCommands.registerCommand("AutoAllignRight", limelight.allignAllAxisLeft());

        autoChooser = AutoBuilder.buildAutoChooser();
       
        shuffleboardData();

        SmartDashboard.putNumber("Tx Value", limelight.getTx());
        
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    DriveConstants.fieldRelative, true),
                m_robotDrive));

        elevator.setDefaultCommand(
            new RunCommand(
                () -> elevator.setManual(
                    -MathUtil.applyDeadband(m_driverController1.getLeftY(), Constants.Elevator.kElevatorDeadband)
                ),elevator
            )
        );

        m_climber.setDefaultCommand(m_climber.setPower(climbConstants.stopClimb));
        //    m_climb.setDefaultCommand(new RunCommand(() -> m_climb.Climb(m_driverController1.getRightY()), m_climb));
        
        //    m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.stop(), m_elevator));
  
        //    m_arm.setDefaultCommand(new RunCommand(()-> m_arm.stop(), m_arm));
  
        //    m_intake.setDefaultCommand(new RunCommand(()-> m_intake.stop(), m_intake));
        //  m_climb.setDefaultCommand(new RunCommand(() -> m_climb.StopClimb(), m_climb)); 
    }

    private void configureButtonBindings() {
    m_driverController.leftTrigger(OIConstants.kDriveDeadband);

    // m_driverController.leftBumper().whileTrue(limelight.allignRobot(limelightConstants.LEFTALLIGN));
    
    // m_driverController.rightBumper().whileTrue(limelight.allignRobot(limelightConstants.RIGHTALLIGN));
    
    // m_driverController.rightTrigger(OIConstants.kTriggerTreshold).whileTrue(limelight.allignRobot(limelightConstants.ALGAEALLIGN));

   
   m_driverController.y().onTrue(m_robotDrive.changeDrivingMode());

    m_driverController.b().onTrue(m_robotDrive.calibrate());

    m_driverController1.a().onTrue(elevator.moveToRestCommand());
    
    m_driverController1.b().onTrue(elevator.moveToL2Command());

    m_driverController1.x().onTrue(elevator.moveToL3Command());

    m_driverController1.y().onTrue(elevator.moveToL4Command());


    m_driverController1.povUp().whileTrue(m_climber.setPower(climbConstants.unflexPower));


    }

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    }

    public void shuffleboardData() {
        ShuffleboardTab  tab = Shuffleboard.getTab("RobotData");
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



