// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSystem extends SubsystemBase {
  public SparkMax elevatorLeftMotor = new SparkMax(Constants.ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
  public SparkMax elevatorRightMotor = new SparkMax(Constants.ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

  private RelativeEncoder elevatorEncoder = elevatorLeftMotor.getEncoder();

  SparkMaxConfig configSparkMotorLeft = new SparkMaxConfig();
  SparkMaxConfig configSparkMotorRight = new SparkMaxConfig();

  private PIDController pidController;
  public ElevatorFeedforward feedforward;

  public ElevatorState stateElevator = ElevatorState.CORAL;
  public ModeElevator modeElevator = ModeElevator.AUTOMATIC;

  public ElevatorSystem() {
    configSparkMotorLeft
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight.follow(Constants.ElevatorConstants.elevatorLeftMotorID, true);

    configSparkMotorLeft.smartCurrentLimit(30);
    configSparkMotorRight.smartCurrentLimit(30);

    elevatorLeftMotor.configure(configSparkMotorLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRightMotor.configure(configSparkMotorRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
    pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

    feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

    pidController.reset();
    zeroElevator();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder", getElevatorEncoderPosition());
    SmartDashboard.putBoolean("Mode", modeElevator.mode);
    SmartDashboard.putNumber("Current Motor", elevatorLeftMotor.getOutputCurrent());

    SmartDashboard.putNumber("PID", pidController.getP());

    double pidOutput = pidController.calculate(getElevatorEncoderPosition(), stateElevator.position);
    double speed = pidOutput;

    if(!modeElevator.mode){
      elevatorLeftMotor.set(speed * Constants.ElevatorConstants.ELEVATOR_SPEED);
    }  
  }

  public void moveElevatorManual(double speed){
    SmartDashboard.putNumber("Speed", speed);
    if(modeElevator.mode && getElevatorEncoderPosition() <= ElevatorState.CORAL.position && speed <= 0){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() >= ElevatorState.L4.position && speed > 0){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() > ElevatorState.CORAL.position && getElevatorEncoderPosition() < ElevatorState.L4.position){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode){
      stopElevator();
    }
  }

  public double getElevatorEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  public void setElevatorEncoderPosition(double position) {
    elevatorEncoder.setPosition(position);
  }

  public void stopElevator(){
    elevatorLeftMotor.stopMotor();
  }

  public void zeroElevator(){
    elevatorEncoder.setPosition(0);
  }

  public void SetCurrentStateElevator(ElevatorState state){
    this.stateElevator = state;
  }

  public void SetCurrentModeElevator(ModeElevator state){
    this.modeElevator = state;
  }
}
