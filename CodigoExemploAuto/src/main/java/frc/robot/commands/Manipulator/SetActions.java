// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;

/** An example command that uses an example subsystem. */
public class SetActions extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Command ClimbPosition(){

      return new SetMechanismState(PivoState.OPEN);
    }

    public Command DeployerAction(){

      return new SetMechanismState(DeployerState.SHOOTING).
          andThen(new WaitCommand(1)).
          andThen(new SetMechanismState(DeployerState.STOPPED));
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
