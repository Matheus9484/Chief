// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Reef;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class Alinhamento3dEsquerda extends Command {
  SwerveSubsystem swerveSubsystem;
  boolean finalizar;
  double velocidadeAlinhamento = 0.12;
  SimpleMotorFeedforward feedforward = Constants.alinhamento.feedforward;
  CommandXboxController xbox;

  PIDController pidY = Constants.alinhamento.pidY;
  PIDController pidX = Constants.alinhamento.pidX;
  PIDController pidController = Constants.alinhamento.angulacao;

  boolean finalizarCronometro;

  double velocidade = 0.2;
  SwerveDrive swerve;
  double outputX;
  double outputY;
  double outputRotacao;
  int[] validIDs;
  private Timer cronometro;
  double timer;
  CommandXboxController controlee = new CommandXboxController(0);

  /** Creates a new SwerveAutoAlinhamento. */
  public  Alinhamento3dEsquerda(SwerveSubsystem swerveSubsystem,  CommandXboxController xbox, int[] validIDs, double timer) {
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
      //new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC));

      finalizar = false;
      //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
      cronometro.reset();
      cronometro.start();

      
    }
    
    @Override
    public void execute() {
      System.out.println("Alinhamento 3D Esquerda");
      System.out.println(cronometro);
      SmartDashboard.getNumber("Timer", cronometro.get());
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
       finalizarCronometro = cronometro.get() > timer;
        if(xbox.button(3).getAsBoolean()){
            finalizar = true;
        }
    
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        boolean tv = LimelightHelpers.getTV("limelight");
        double fiducialID = LimelightHelpers.getFiducialID("limelight");
        System.out.println(fiducialID);
        if(!tv && Math.abs(controlee.getRawAxis(1)) > 0){
          finalizar = true;
        }
    
        swerveSubsystem.visao = false;
        
        if (pose != null) {
            double roll = normalizeAngle(pose[5]);
    
            // Alinhar o eixo X primeiro
            outputY = pidX.calculate(tx, Constants.alinhamento.setPointYEsquerda);
            
            // Alinhar a rotação depois do eixo X
            outputRotacao = pidController.calculate(roll, Constants.alinhamento.setPointRotationLE);
            
            // Alinhar o eixo Y por último
            outputX = pidY.calculate(ty, Constants.alinhamento.setPointXEsquerda);
    
            LimelightHelpers.getTargetPose_RobotSpace("limelight");
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
    swerveSubsystem.visao = false;
    finalizar = false;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Constants.Tags.allTags);
  }

  //condição para finalizar
  @Override
  public boolean isFinished() {
     double tx = LimelightHelpers.getTX("limelight");
     double ty = LimelightHelpers.getTY("limelight");
     double[] pose = LimelightHelpers.getTargetPose_RobotSpace("limelight"); 
     double yaw = pose[5];

     boolean alinhadoX = Math.abs(tx - Constants.alinhamento.setPointYEsquerda) < 1.0;   
     boolean alinhadoY = Math.abs(ty - Constants.alinhamento.setPointXEsquerda) < 2;   
     boolean alinhadoRot = Math.abs(yaw - Constants.alinhamento.setPointRotationLE) < 2.0; 

     return LimelightHelpers.getTV("limelight") && alinhadoX && alinhadoY && alinhadoRot || finalizar || finalizarCronometro;
    }    
  
  //normaliza o angulo uma vez que ele lê negativo ao inves do 360 completo.
  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }
}
