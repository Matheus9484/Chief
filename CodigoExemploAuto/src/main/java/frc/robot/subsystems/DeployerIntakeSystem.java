// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.IntakeState;
import frc.robot.commands.Manipulator.GetSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeployerIntakeSystem extends SubsystemBase {
  public SparkFlex deployerLeftMotor = new SparkFlex(Constants.DeployerIntakeConstants.deployerLeftMotorID, MotorType.kBrushless);
  public SparkFlex deployerRightMotor = new SparkFlex(Constants.DeployerIntakeConstants.deployerRightMotorID, MotorType.kBrushless);

  SparkFlexConfig configSparkMotorLeft = new SparkFlexConfig();
  SparkFlexConfig configSparkMotorRight = new SparkFlexConfig();

  public GetSensor getSensor = new GetSensor();

  public DeployerState stateDeployer;
  public IntakeState stateIntake = IntakeState.STOPPED;

  public final DigitalInput deployerSensor1;
  public final DigitalInput deployerSensor2;
  public  final DigitalInput funnelSensor;

  public DeployerIntakeSystem() {
    configSparkMotorLeft
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight.follow(Constants.DeployerIntakeConstants.deployerLeftMotorID, true);

    deployerLeftMotor.configure(configSparkMotorLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    deployerRightMotor.configure(configSparkMotorRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployerSensor1 = new DigitalInput(Constants.DeployerIntakeConstants.deployerSensor1);
    deployerSensor2 = new DigitalInput(Constants.DeployerIntakeConstants.deployerSensor2);
    funnelSensor = new DigitalInput(Constants.DeployerIntakeConstants.funnelSensor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("DeployerSensor1", getSensor.getDigitalSensor(deployerSensor1));
    SmartDashboard.putBoolean("DeployerSensor2", getSensor.getDigitalSensor(deployerSensor2));
    SmartDashboard.putBoolean("FunnelSensor", getSensor.getDigitalSensor(funnelSensor));
    SmartDashboard.putNumber("Current Elevator", deployerLeftMotor.getOutputCurrent());

    // if(stateDeployer == DeployerState.SHOOTING || stateIntake.operation){
    //   deployerLeftMotor.set(stateDeployer.speed);
    // } else{
    //   deployerLeftMotor.stopMotor();
    // }
  }

  public boolean HasPieceDeployer(){
    if(getSensor.getDigitalSensor(deployerSensor1) || getSensor.getDigitalSensor(deployerSensor2)){
      return true;
    } else {
      return false;
    }
  }
 

  public void SetCurrentStateDeployer(DeployerState state){
    this.stateDeployer = state;
  }

  public void SetCurrentStateIntake(IntakeState state){
    this.stateIntake = state;
  }
 }
