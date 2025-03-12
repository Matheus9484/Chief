// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;


public class SwerveLadoEsquerda extends Command {
 
  CommandXboxController xbox;
  boolean ligado;
  SwerveSubsystem subsystem;
  SwerveDrive swerveDrive;

  public SwerveLadoEsquerda(CommandXboxController controleXbox, SwerveSubsystem swerveSubsystem) {
    this.xbox = controleXbox;
    this.subsystem = swerveSubsystem; 
    this.swerveDrive = subsystem.swerveDrive;

    addRequirements(swerveSubsystem);
  }

  //Comando que roda quando é chamado.
  @Override
  public void initialize() {
    SwerveDriveTest.angleModules(swerveDrive, Constants.SwerveConfigs.graus90);
    Timer.delay(0.09);
    ligado = false;
  }

  //executa lateralmente o swerve.
  @Override
  public void execute() {
    double velocidadeLateral = 0.1;  
    swerveDrive.drive(new Translation2d(0, -velocidadeLateral * swerveDrive.getMaximumChassisVelocity()), 0, false, false);
  }
  
  //finaliza o comando quando é interrompido.
  @Override
  public void end(boolean interrupted) {
    ligado = true;
  }
   
  //retorna verdadeiro quando o comando é finalizado.
  @Override
  public boolean isFinished() {
    return ligado;
  }
}
