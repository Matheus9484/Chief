// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;

/** An example command that uses an example subsystem. */
public class SetMechanismState extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DeployerState currentStateDeployer = DeployerState.STOPPED;
    private IntakeState currentStateIntake = IntakeState.STOPPED;

    private ElevatorState currentStateElevator = ElevatorState.L1;
    private ModeElevator currentModeElevator = ModeElevator.AUTOMATIC;

    private PivoState currentStatePivo = PivoState.OPEN;

    private ClimberState currentStateClimber = ClimberState.STOPPED;

    //Only

    boolean currentStateDeployerOnly = false;
    boolean currentStateIntakeOnly = false;

    boolean currentStateElevatorOnly = false;
    boolean currentModeElevatorOnly = false;

    boolean currentStatePivoOnly = false;

    boolean currentStateClimberOnly = false;

    public SetMechanismState(ElevatorState state) {
      addRequirements(RobotContainer.elevatorSystem);
      this.currentStateElevator = state;
      currentStateElevatorOnly = true;
    }

    public SetMechanismState(ModeElevator state) {
      addRequirements(RobotContainer.elevatorSystem);
      this.currentModeElevator = state;
      currentModeElevatorOnly = true;
    }

    public SetMechanismState(DeployerState state) {
      addRequirements(RobotContainer.deployerIntakeSystem);
      this.currentStateDeployer = state;
      currentStateDeployerOnly = true;
    }

    public SetMechanismState(IntakeState state) {
      addRequirements(RobotContainer.deployerIntakeSystem);
      this.currentStateIntake = state;
      currentStateIntakeOnly = true;
    }

    public SetMechanismState(PivoState state) {
      addRequirements(RobotContainer.pivoSystem);
      this.currentStatePivo = state;
      currentStatePivoOnly = true;
    }

    public SetMechanismState(ClimberState state) {
      addRequirements(RobotContainer.climberSystem);
      this.currentStateClimber = state;
      currentStateClimberOnly = true;
    }

  @Override
  public void execute() {
    if(currentStateElevatorOnly){
      if(RobotContainer.deployerIntakeSystem.HasPieceDeployer()){
        RobotContainer.elevatorSystem.SetCurrentStateElevator(this.currentStateElevator);
      } else if(!RobotContainer.deployerIntakeSystem.HasPieceDeployer() && currentStateElevator == ElevatorState.CORAL){
        RobotContainer.elevatorSystem.SetCurrentStateElevator(this.currentStateElevator);
      }
    } else if(currentModeElevatorOnly){
      RobotContainer.elevatorSystem.SetCurrentModeElevator(this.currentModeElevator);
    }

    if(currentStateElevatorOnly){
        RobotContainer.elevatorSystem.SetCurrentStateElevator(this.currentStateElevator);
    } else if(currentModeElevatorOnly){
      RobotContainer.elevatorSystem.SetCurrentModeElevator(this.currentModeElevator);
    }

    else if(currentStateDeployerOnly) {
      RobotContainer.deployerIntakeSystem.SetCurrentStateDeployer(this.currentStateDeployer);
    } else if(currentStateIntakeOnly){
      RobotContainer.deployerIntakeSystem.SetCurrentStateIntake(this.currentStateIntake);
    } 
    
    if(currentStatePivoOnly){
      RobotContainer.pivoSystem.SetCurrentStatePivo(this.currentStatePivo);
    }
    
    else if(currentStateClimberOnly){
      RobotContainer.climberSystem.SetCurrentStateClimber(this.currentStateClimber);
    }
  }
}
