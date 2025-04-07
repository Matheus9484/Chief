// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.time.format.SignStyle;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeState;
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.PivoState;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeSystem extends SubsystemBase {
  public SparkMax algaeMotor = new SparkMax(Constants.AlgaeConstants.algaeMotorID, MotorType.kBrushless);

  SparkMaxConfig configSparkMotor = new SparkMaxConfig();
  public AlgaeState stateAlgae = AlgaeState.STOPPED;

  public AlgaeSystem() {
    configSparkMotor
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    configSparkMotor.smartCurrentLimit(20);

    algaeMotor.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void periodic() {
    SmartDashboard.putNumber("Current Motor ALGAE", algaeMotor.getOutputCurrent());
    
    if(stateAlgae == AlgaeState.GET){
        algaeMotor.set(stateAlgae.speed);
    } else if(stateAlgae == AlgaeState.SHOOT){
        algaeMotor.set(stateAlgae.speed);
    } else{
        algaeMotor.stopMotor();
    }
  }

  public void SetCurrentStateAlgae(AlgaeState state){
    this.stateAlgae = state;
  }
}
