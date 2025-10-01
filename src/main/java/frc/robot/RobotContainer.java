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
import frc.robot.Subsystems.Elevator;
// import frc.robot.Subsystems.ElevatorSubsystem;
// import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_driverController1 = new XboxController(OIConstants.kDriverController1Port);

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

        //    m_climb.setDefaultCommand(new RunCommand(() -> m_climb.Climb(m_driverController1.getRightY()), m_climb));
        
        //    m_elevator.setDefaultCommand(new RunCommand(()-> m_elevator.stop(), m_elevator));
  
        //    m_arm.setDefaultCommand(new RunCommand(()-> m_arm.stop(), m_arm));
  
        //    m_intake.setDefaultCommand(new RunCommand(()-> m_intake.stop(), m_intake));
        //  m_climb.setDefaultCommand(new RunCommand(() -> m_climb.StopClimb(), m_climb)); 
    }

    private void configureButtonBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(new InstantCommand(() ->m_robotDrive.changeSpeed(), m_robotDrive));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.05).whileTrue(limelight.allignAllAxisRight());
    
    new Trigger(()-> m_driverController.getLeftTriggerAxis() >0.05).whileTrue(limelight.allignAllAxisLeft());

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .onTrue(new InstantCommand(()-> m_robotDrive.changeDrivingMode(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .onTrue(new InstantCommand(()-> m_robotDrive.calibrate(), m_robotDrive)); 

    new JoystickButton(m_driverController1, XboxController.Button.kA.value)
    .onTrue(elevator.moveToRestCommand());

    new JoystickButton(m_driverController1, XboxController.Button.kX.value)
    .onTrue(elevator.moveToL2Command());

    new JoystickButton(m_driverController1, XboxController.Button.kY.value)
    .onTrue(elevator.moveToL3Command());

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



