// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants{


  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.4; // radians per second   original es 1.2
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)  original es 1.8
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)   original es 2.0

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 14;

    public static final int kFrontLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 22;
    public static final int kRearRightTurningCanId = 23;
    public static final int kRearLeftTurningCanId = 24;

    public static final boolean kGyroReversed = true;
    public static boolean fieldRelative = true;
    public static float deltaangle = 0;

    public static boolean kSlowMode = false;


  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }



public static class ElevatorConstants
{
  public static final int kElevatorMotorCanId = 41;
    public static final boolean kElevatorMotorInverted = true;

  public static final double   kElevatorKP              = 40.257;
  public static final double   kElevatorKI              = 0;
  public static final double   kElevatorKD              = 8.565;
  public static final double   kElevatorkS              = 0.26737; // volts (V)
  public static final double   kElevatorkV              = 9.9682;//10.773; // volt per velocity (V/(m/s))
  public static final double   kElevatorkA              = 0.1931; // volt per acceleration (V/(m/s²))
  public static final double   kElevatorkG              = 0.39266; // volts (V)
  public static final double   kElevatorGearing         = 24.0;
  public static final double   kElevatorSproketTeeth    = 16;
  public static final double   kElevatorPitch           = Units.inchesToMeters(0.625);
  public static final double   kElevatorDrumRadius      = (kElevatorSproketTeeth * kElevatorPitch) / (2 * Math.PI);// radius = Circumference / (2 pi)
  public static final double   kCarriageMass            = 5.7; // kg
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double   kElevatorUnextendedHeightMeters   = Units.inchesToMeters(40.9949);
  public static final Distance kElevatorUnextendedHeight = Meters.of(kElevatorUnextendedHeightMeters); 
  public static final double   kMinElevatorHeightMeters = Units.inchesToMeters(0);//min height / 10
  public static final double   kMaxElevatorHeightMeters = Units.inchesToMeters(23.09958818897638)+  kElevatorUnextendedHeightMeters;
  
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final Distance kLaserCANOffset          = Meters.of(0.27);

  //public static final double kElevatorMaxVelocity = 3.5;
  //public static final double kElevatorMaxAcceleration = 2.5;

  public static final Distance kMinElevatorHeight      = Meters.of(kMinElevatorHeightMeters);
  public static final Distance kMaxElevatorHeight      = Meters.of(kMaxElevatorHeightMeters);
  public static final double   kElevatorAllowableError = Units.inchesToMeters(0.2);
  // public static final double L1 = Units.inchesToMeters(); //la posición oficial es 13
  public static final double BasePosition = Units.inchesToMeters(42);
  public static final double L2 = Units.inchesToMeters(42.5);  //la posición oficial es 31.875
  public static final double L3 = Units.inchesToMeters(58.6614); //la posición oficial es 47.625
  // public static final double L4 = Units.inchesToMeters(70.55); // la posición oficial es 72
  public static final double TopAlgaeHeight = Units.inchesToMeters(60);
  public static final double BottomAlgaeHeight = Units.inchesToMeters(50);

  public static       int      elevatorMotorID         = 41;

  
  public static       double   kElevatorRampRate       = 0.1;
  public static       int      kElevatorCurrentLimit   = 40;
  public static double kMaxVelocity = Meters.of(2).per(Second).in(MetersPerSecond);
  public static double kMaxAcceleration = Meters.of(1).per(Second).per(Second).in(MetersPerSecondPerSecond);

  public static boolean kElevatorAutomatic = true;


  }

 

public static  class limelightConstants{

  // how many degrees back is your limelight rotated from perfectly vertical?
public static double limelightMountAngleDegrees = 15.0; 

// distance from the center of the Limelight lens to the floor
public static double limelightLensHeightInches = Units.metersToInches(0.23); 

// distance from the target to the floor
public static double goalHeightInches = Units.metersToInches(0.305); 



public static boolean isRightReef;
public static  double xReefSetpoint = isRightReef? 3: -3;

public static final double yReefSetpoint =  3;
public static final double yReefTolerance = 0.15;

public static final double xLeftReefSetpoint = 4;
public static final double xRightReefSetpoint = -7;
public static final double xReefTolerance = 0.15;

public static final double rotReefSetpoint = 0.0;
public static final double rotReefTolerance = 1;



}

public static class NeoMotorConstants {
public static final double kFreeSpeedRpm = 5676;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;
  public static final double kDriveDeadband = 0.02;
  public static final double kTriggerTreshold = 0.1;
  
  public static final int kDriverController1Port = 1;
  // public static final double kElevatorDeadband = 0.02;
  public static final double kArmDeadband = 0.01 ;
  public static final double kIntakeDeadband = 0.02;
}

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
}


  


 
  



    

