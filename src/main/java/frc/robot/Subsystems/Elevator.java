// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {
  public enum State {
    IDLE,
    MOVING,
    MANUAL
    }

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController pid;

  private State state = State.IDLE;
  private double setpointMeters = Constants.Elevator.LowPOS;

  double positionFactor = (2 * Math.PI * Constants.Elevator.DrumRadius) / Constants.Elevator.Gearing;


  public Elevator() {
    motor = new SparkMax(Constants.Elevator.ElevatorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(true);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    encoder.setPosition(Constants.Elevator.LowPOS); // Asumir inicio en bajo

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints( // especializado para subir, estabilizar, despues bajar velocidad
        Constants.Elevator.MAX_VEL_M_S,
        Constants.Elevator.MAX_ACCEL_M_S2);

    pid = new ProfiledPIDController(
      Constants.Elevator.kP, 
      Constants.Elevator.kI, 
      Constants.Elevator.kD, 
      constraints);
    pid.setTolerance(0.01);
    pid.reset(getPositionMeters());
    
    SmartDashboard.putNumber("Elevator/kP", Constants.Elevator.kP);
    SmartDashboard.putNumber("Elevator/kI", Constants.Elevator.kI);
    SmartDashboard.putNumber("Elevator/kD", Constants.Elevator.kD);
  }


  public double getPositionMeters() {
    return encoder.getPosition() * positionFactor;
  }

  public double getVelocityMetersPerSecond() {
    return encoder.getVelocity() * positionFactor;
  }

  public void setManual(double output) {
    state = State.MANUAL;
    output = Math.max(Constants.Elevator.MIN_OUTPUT, Math.min(Constants.Elevator.MAX_OUTPUT, output));

    // Soft limits
    if (getPositionMeters() <= Constants.Elevator.MINheight && output < 0) output = 0;
    if (getPositionMeters() >= Constants.Elevator.MAXheight && output > 0) output = 0;

    motor.set(output);
  }

  public void moveToPosition(double targetMeters) {
    setpointMeters = Math.max(Constants.Elevator.MINheight,
            Math.min(Constants.Elevator.MAXheight, targetMeters));
    pid.reset(getPositionMeters());
    state = State.MOVING;
  }

  @Override
  public void periodic() {
  // Actualizar PID desde dashboard
  pid.setP(SmartDashboard.getNumber("Elevator/kP", Constants.Elevator.kP));
  pid.setI(SmartDashboard.getNumber("Elevator/kI", Constants.Elevator.kI));
  pid.setD(SmartDashboard.getNumber("Elevator/kD", Constants.Elevator.kD));
    
  switch (state) {
    case MOVING:
        double output = pid.calculate(getPositionMeters(), setpointMeters);
         // Limitar velocidad máxima
        output = Math.max(Constants.Elevator.MIN_OUTPUT, Math.min(Constants.Elevator.MAX_OUTPUT, output));
        motor.set(output);
    
        if (pid.atGoal()) state = State.IDLE;
        break;
    case IDLE:
        motor.set(0);
        break;
    case MANUAL:
        break;
    }
    
    SmartDashboard.putNumber("Elevator/PositionMeters", getPositionMeters());
    SmartDashboard.putNumber("Elevator/SetpointMeters", setpointMeters);
    SmartDashboard.putString("Elevator/State", state.name());
  }

  public Command moveToRestCommand() {
        return Commands.runOnce(() -> moveToPosition(Constants.Elevator.LowPOS), this)
                       .andThen(Commands.waitUntil(() -> pid.atGoal()));
    }

    public Command moveToL2Command() {
        return Commands.runOnce(() -> moveToPosition(Constants.Elevator.L2), this)
                       .andThen(Commands.waitUntil(() -> pid.atGoal()));
    }

    public Command moveToL3Command() {
        return Commands.runOnce(() -> moveToPosition(Constants.Elevator.L3), this)
                       .andThen(Commands.waitUntil(() -> pid.atGoal()));
    }
}
