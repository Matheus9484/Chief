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
import frc.robot.Constants.PivoState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivoSystem extends SubsystemBase {
  public SparkMax pivoMotor = new SparkMax(Constants.PivoConstants.pivoMotorID, MotorType.kBrushless);

  private RelativeEncoder pivoEncoder = pivoMotor.getEncoder();

  public PIDController pidController;
  public ElevatorFeedforward feedforward;

  SparkMaxConfig configSparkMotor = new SparkMaxConfig();
  public PivoState statePivo = PivoState.CLOSED;

  public PivoSystem() {
    configSparkMotor
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    configSparkMotor.smartCurrentLimit(30);

    pivoMotor.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = new PIDController(Constants.PivoConstants.kP, Constants.PivoConstants.kI, Constants.PivoConstants.kD);
    pidController.setTolerance(Constants.PivoConstants.PID_TOLERANCE);

    feedforward = new ElevatorFeedforward(Constants.PivoConstants.kS, Constants.PivoConstants.kG, Constants.PivoConstants.kV);

    zeroPivo();
    pidController.reset();
  }

  public void periodic() {
    SmartDashboard.putNumber("Encoder PIVO", getPivoEncoderPosition());
    SmartDashboard.putNumber("Current Motor PIVO", pivoMotor.getOutputCurrent());

    double pidOutput = pidController.calculate(pivoMotor.getEncoder().getPosition(), statePivo.position);
    double feedforwardOutput = feedforward.calculate(pivoMotor.getEncoder().getPosition(), pivoMotor.getEncoder().getVelocity());
    double speed = pidOutput + feedforwardOutput;

    pivoMotor.set(speed * Constants.PivoConstants.PIVO_SPEED);
  }

  public double getPivoEncoderPosition() {
    return pivoEncoder.getPosition();
  }

  public void zeroPivo(){
    pivoEncoder.setPosition(0);
  }

  public void SetCurrentStatePivo(PivoState state){
    this.statePivo = state;
  }
}
