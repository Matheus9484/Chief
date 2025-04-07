// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorState;

import frc.robot.Constants.ModeElevator;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSystem extends SubsystemBase {
  private TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.elevatorLeftMotorID);
  private TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.elevatorRightMotorID);

  MotionMagicExpoVoltage motionRequest;

  public ElevatorState stateElevator = ElevatorState.CORAL;
  public ModeElevator modeElevator = ModeElevator.MANUAL;
  // private TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.elevatorRightMotorID);

  // public SparkMax elevatorLeftMotor = new SparkMax(Constants.ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
  // public SparkMax elevatorRightMotor = new SparkMax(Constants.ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

  // SparkMaxConfig configSparkMotorLeft = new SparkMaxConfig();
  // SparkMaxConfig configSparkMotorRight = new SparkMaxConfig();

  // private PIDController pidController;
  //public ElevatorFeedforward feedforward;

  // private double targetVelocity = 0.0; // Velocidade desejada
  // private double targetAcceleration = 0.0; // Aceleração desejada
  
  // private final ElevatorFeedforward feedforward = 
  // new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV, Constants.ElevatorConstants.kA);

  public ElevatorSystem() {

    elevatorLeftMotor.getConfigurator().apply(Constants.ELEVATOR_CONFIG);
    elevatorRightMotor.getConfigurator().apply(Constants.ELEVATOR_CONFIG);

    elevatorRightMotor.setControl(new Follower(Constants.ElevatorConstants.elevatorLeftMotorID, true));

    motionRequest = new MotionMagicExpoVoltage(0);

    // pidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
    // pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

    // pidController.reset();
    zeroElevator();
  }



  @Override
  public void periodic() {

      if(!modeElevator.mode){
        elevatorLeftMotor.setControl(motionRequest.withPosition(stateElevator.position));
        // elevatorLeftMotor.setVoltage(totalOutput);
      }  

        // double currentPosition = getElevatorEncoderPosition();
        // double currentVelocity = getTargetVelocity();

        // // Cálculo do Feedforward para compensar gravidade
        // double feedforwardOutput = feedforward.calculate(currentVelocity);

        // // Cálculo do PID para atingir a posição alvo
        // double pidOutput = pidController.calculate(currentPosition, stateElevator.position);

        // // Define a voltagem do motor combinando Feedforward + PID
        // double totalOutput = feedforwardOutput + pidOutput;

        //kV = 1,625



    // Primeira coisa a se testar, objetivo e descobrir a tensão minima que o elevador começa a se mover
        // elevatorLeftMotor.setVoltage(0.4);

        //segunda coisa descobrir O KV

        // System.out.println("Velocidade: " + getTargetAcceleration());

        // Enviar dados para o SmartDashboard para monitoramento

        // SmartDashboard.putNumber("Feedforward", feedforwardOutput);
        // SmartDashboard.putNumber("PID Output", pidOutput);
        // SmartDashboard.putNumber("Saída Final", totalOutput);

        SmartDashboard.putNumber("EncoderElevator", getElevatorEncoderPosition());
        SmartDashboard.putNumber("Posição Alvo", stateElevator.position);

        System.out.println(elevatorLeftMotor.getSupplyCurrent());
  }

  public void moveElevatorManual(double speed){
    SmartDashboard.putNumber("Speed", speed);
    if(modeElevator.mode && getElevatorEncoderPosition() <= 5 && speed <= 0){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() >= 195 && speed > 0){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode && getElevatorEncoderPosition() > 5 && getElevatorEncoderPosition() < 195){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode){
      stopElevator();
    }
  }

  public double getElevatorEncoderPosition() {
    return elevatorRightMotor.getPosition().getValueAsDouble();
  }

  public void setElevatorEncoderPosition(double position) {
    elevatorRightMotor.setPosition(position);
  }

  public void stopElevator(){
    elevatorLeftMotor.stopMotor();
  }

  public void zeroElevator(){
    elevatorRightMotor.setPosition(0);
  }

  public void SetCurrentStateElevator(ElevatorState state){
    this.stateElevator = state;
  }

  public void SetCurrentModeElevator(ModeElevator state){
    this.modeElevator = state;
  }

}
