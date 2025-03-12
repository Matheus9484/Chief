// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Reef;


import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class Alinhamento3dDireita
 extends Command {
  SwerveSubsystem swerveSubsystem;
  boolean finalizar;
  double velocidadeAlinhamento = 0.12;
  SimpleMotorFeedforward feedforward = Constants.alinhamento.feedforward;
  CommandXboxController xbox;

  PIDController pidY = Constants.alinhamento.pidY;
  PIDController pidX = Constants.alinhamento.pidX;
  PIDController pidController = Constants.alinhamento.angulacao;

  double velocidade = 0.2;
  SwerveDrive swerve;
  double outputX;
  double outputY;
  double outputRotacao;
  int[] validIDs;
  private Timer cronometro;
  boolean finalizarCronometro;
  double timer;
  

  /** Creates a new SwerveAutoAlinhamento. */
  public Alinhamento3dDireita(SwerveSubsystem swerveSubsystem,  CommandXboxController xbox, int[] validIDs, double timer) {
    this.swerveSubsystem = swerveSubsystem;
    this.xbox = xbox;
    this.validIDs = validIDs;
    this.timer = timer;
    cronometro = new Timer();

    pidController.enableContinuousInput(-180, 180);
    
    addRequirements(swerveSubsystem);
  }
  
  @Override
  public void initialize() {
    finalizar = false;
    cronometro.reset();
    cronometro.start();
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", validIDs);
    }
    
    @Override
    public void execute() {

      finalizarCronometro = cronometro.get() > timer;

        if(xbox.button(3).getAsBoolean()){
            finalizar = true;
        }
    
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight-direita");
        double tx = LimelightHelpers.getTX("limelight-direita");
        double ty = LimelightHelpers.getTY("limelight-direita");
        boolean tv = LimelightHelpers.getTV("limelight-direita");
        double fiducialID = LimelightHelpers.getFiducialID("limelight-direita");
        System.out.println(fiducialID);
    
        swerveSubsystem.visao = false;
        
        if (pose != null) {
            double roll = normalizeAngle(pose[5]);
    
            // Alinhar o eixo X primeiro
            outputY = pidX.calculate(tx, Constants.alinhamento.setPointYDireita);
            
            // Alinhar a rotação depois do eixo X
            outputRotacao = pidController.calculate(roll, Constants.alinhamento.setPointRotationLD);
            
            // Alinhar o eixo Y por último
            outputX = pidY.calculate(ty, Constants.alinhamento.setPointXDireita);
    
            LimelightHelpers.getTargetPose_RobotSpace("limelight-direita");
            System.out.println("Roll: " + roll);
    
            if (!tv) {
                swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
            } else {
                swerveSubsystem.drive(new Translation2d(-outputX, -outputY), -outputRotacao, false);
            }
        } else {
            swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
        }
    }
    

  //codigo que roda quando o comando é interrompido.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    swerveSubsystem.visao = true;
    finalizar = false;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Constants.Tags.allTags);
  }

  //condição para finalizar
  @Override
  public boolean isFinished() {
     double tx = LimelightHelpers.getTX("limelight-direita");
     double ty = LimelightHelpers.getTY("limelight-direita");
     double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight-direita"); 
     double yaw = pose[5];

     boolean alinhadoX = Math.abs(tx - Constants.alinhamento.setPointYDireita) < 1.0;   
     boolean alinhadoY = Math.abs(ty - Constants.alinhamento.setPointXDireita) < 2;   
     boolean alinhadoRot = Math.abs(yaw - Constants.alinhamento.setPointRotationLD) < 2.0; 

     return LimelightHelpers.getTV("limelight-direita") && alinhadoX && alinhadoY && alinhadoRot || finalizar || finalizarCronometro;
    }    
  
  //normaliza o angulo uma vez que ele lê negativo ao inves do 360 completo.
  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }
}
