// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberState;

import frc.robot.Constants.PivoState;

public class ClimberSystem extends SubsystemBase {
  public SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);

  SparkMaxConfig configSparkMotor = new SparkMaxConfig();

  //private RelativeEncoder climberEncoder = climberMotor.getEncoder();

  public ClimberState stateClimber = ClimberState.STOPPED;

  public ClimberSystem() {
    configSparkMotor
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    configSparkMotor.smartCurrentLimit(80);

    climberMotor.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberMotor.getEncoder().setPosition(0);
  }

  public void periodic() {
    SmartDashboard.putNumber("Encoder Climber", climberMotor.getEncoder().getPosition());

    if(RobotContainer.pivoSystem.statePivo == PivoState.OPEN && stateClimber == ClimberState.OPENING && climberMotor.getEncoder().getPosition() < 600){
        climberMotor.set(stateClimber.speed);
    } else if(stateClimber == ClimberState.CLOSING && climberMotor.getEncoder().getPosition() > 0){
        climberMotor.set(stateClimber.speed);
    } else{
        climberMotor.stopMotor();
    }
  }

  public void SetCurrentStateClimber(ClimberState state){
    this.stateClimber = state;
  }
}
