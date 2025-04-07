// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;


public class SwerveLadoEsquerdaAuto extends Command {
 
  CommandXboxController xbox;
  boolean ligado;
  SwerveSubsystem subsystem;
  SwerveDrive swerveDrive;
  
  boolean finalizarCronometro;
  private Timer cronometro;
  double timer;

  public SwerveLadoEsquerdaAuto(CommandXboxController controleXbox, SwerveSubsystem swerveSubsystem, double timer) {
    this.xbox = controleXbox;
    this.subsystem = swerveSubsystem; 
    this.swerveDrive = subsystem.swerveDrive;
    this.timer = timer;
    cronometro = new Timer();

    addRequirements(swerveSubsystem);
  }

  //Comando que roda quando é chamado.
  @Override
  public void initialize() {
    SwerveDriveTest.angleModules(swerveDrive, Constants.SwerveConfigs.graus90);
    Timer.delay(0.09);
    ligado = false;

    cronometro.reset();
    cronometro.start();
    finalizarCronometro = false;
  }

  //executa lateralmente o swerve.
  @Override
  public void execute() {
    finalizarCronometro = cronometro.get() > timer;

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
    return ligado || finalizarCronometro;
  }
}
