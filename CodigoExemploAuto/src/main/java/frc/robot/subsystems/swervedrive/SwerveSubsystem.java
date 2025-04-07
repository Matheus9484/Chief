// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import frc.robot.Constants.SwerveConfigs;
import frc.robot.LimelightHelpers;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** 
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */

public class SwerveSubsystem extends SubsystemBase {
    // Objeto global da SwerveDrive (Classe YAGSL).
    public SwerveDrive swerveDrive;

    //Instanciamento do pigeon.
    private final Pigeon2 pigeonIMU = new Pigeon2(9);

    //odometria por visao.
    public boolean visao = true;

    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Dimensoes.MAX_SPEED);
        } catch (Exception e) {
          throw new RuntimeException(e);
        }
       
        swerveDrive.setHeadingCorrection(Constants.SwerveConfigs.headingCorrection); //Correcao da direcao com pigeon.
        swerveDrive.setChassisDiscretization(true, Constants.SwerveConfigs.dt); //Para correcao de velocidade.
        swerveDrive.angularVelocityCorrection =  SwerveConfigs.usarCorrecaoDesvioVelocidadeAngular; //Ativar a compensacao angular que acontece quando rotacionamos e translacionamos nosso robo ao mesmo tempo.
        swerveDrive.angularVelocityCoefficient = SwerveConfigs.coeficienteCorrecaoAngVel; //Valor da compensacao angular.
        swerveDrive.setCosineCompensator(SwerveConfigs.cosineCompensator); // Melhora a precisao dos movimentos em angulos.
        swerveDrive.setModuleEncoderAutoSynchronize(SwerveConfigs.moduleEncoderAutoSynchronize, SwerveConfigs.moduleDeadBandEncoderValue); // Sicroniza os modulos do encoder
        SwerveDriveTest.angleModules(swerveDrive,new Rotation2d(1,0)); // seta posicao da roda
        Timer.delay(0.4);  
    
        setupPathPlanner();
    }
    
    @Override
    public void periodic() {
     
      //Atualiza a odometria do swerve 
      swerveDrive.updateOdometry();
    
    }

      public void setupPathPlanner() {
    

    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1.25, 0.0, 0.02),
              // Translation PID constants
              new PIDConstants(2.0, 0.0, 0.015)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }
  

  //Movimenta o robô com o joystick esquerdo, e mira o robo no ângulo no qual o joystick está apontando
  public Command driveCommandAlinharComJoystick(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      // Faz o robô se mover
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  //Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      // Faz o robô se mover
      swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumChassisVelocity(),
                                          yInput * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

    public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

    // Função drive que chamamos em nossa classe de comando Teleoperado.
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) 
    {
      swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad.
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
      getHeading().getRadians());
    }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) 
  {
    return new ChassisSpeeds(xInput, yInput, 0);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo).
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  // Retorna a velocidade relativa ao campo.
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve.
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual e usado para acessar velocidade maxima.
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  // Angulo atual do robô.
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  // Reseta a odometria para uma posição indicada (Usado no autonomo).
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autonomo).
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  // Seta o motor como Brake.
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  //Pega a velocidade do robo.
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  //Comando para parar o robo (retorna void);
  public void stop()
  {
    swerveDrive.drive(new Translation2d(0,0), 0, false, false);
  }

  //Comando para parar o robo (retorna command).
  public Command stopMovement() {
    return run(() -> {
        swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
    });
  }

  // Criação do comando de geração do autonomous.
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  //comando que move o robô lateralmente.
  public Command moveLateralByButton() {
    return run(() -> {
        // Movimento lateral no eixo Y
        double velocidadeLateral = 0.7;  // Velocidade máxima ou um valor fixo
        // O valor de `0` no primeiro parâmetro é para não mover no eixo X (frente/trás)
        System.out.println("Movimento Lateral Iniciado, Velocidade: ");
        swerveDrive.drive(new Translation2d(0, velocidadeLateral * swerveDrive.getMaximumChassisVelocity()), 0, true, false);
    });
}
  
   //atualiza a odometria através da visão;
   public void updateVisionOdometry() {
    if(visao == false){
      return;
    } 

     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-direita");
     if (limelightMeasurement.tagCount == 0) {
       return;
      }
       Pose2d limelightPose = limelightMeasurement.pose;

        
        Transform2d limelightToRobot = new Transform2d(
          new Translation2d(0.3, -0.3),  
          Rotation2d.fromDegrees(22)    
        );
        
        Pose2d correctedPose = limelightPose.transformBy(limelightToRobot.inverse());

        swerveDrive.addVisionMeasurement(correctedPose, limelightMeasurement.timestampSeconds, VecBuilder.fill(0, 0, 9999));
    }

   //pega a rotação do giroscopio.
   public Rotation2d getGyroscopeRotation() {
    return pigeonIMU.getRotation2d();
  }
  
   //Zera o gyro.
   public void zeroGyro() {

    pigeonIMU.setYaw(0);
    resetOdometry(new Pose2d(new Translation2d(0., 0), new Rotation2d(0)));
}

  //Dirige para a posicao escolhida.
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints( 
        swerveDrive.getMaximumChassisVelocity() * 2, 6,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(500));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0.25) // Goal end velocity in meters/sec
                                     );
  }
}
