// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

/**
 * Classe de constantes
 */
public final class Constants {
  // Aqui temos várias constantes referentes as demais áreas do robô
    
  public static final class Dimensoes {
    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double ROBOT_MASS = 38;
    //Velocidade máxima *DEVE SER CONFIGURADO PARA O SEU ROBÔ*
    public static final double MAX_SPEED = 4;
    //Posição do módulo mais longe *COLOQUE OS MESMOS VALORES DO JSON*
    private static final Translation2d FURTHEST_MODULE_POSE = new Translation2d(MAX_SPEED, LOOP_TIME);
    public static final double MAX_ANGULAR_SPEED = SwerveMath.calculateMaxAngularVelocity(MAX_SPEED, FURTHEST_MODULE_POSE.getX(), FURTHEST_MODULE_POSE.getY());

    //Posições do centro de massa *DEVE SER CONFIGURADO PARA SEU ROBÔ*
    private static final double xMass = 0;
    private static final double yMass = 0;
    private static final double zMass = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    // Contem a porta em que o controle está
    public static final class Controle {
      // Porta do controle
      public static final int xboxControle = 0;
      
      public static final int buttonBox = 1;
      
      public static final int joyManual = 2;
      
      // Deadband do controle
      public static final double DEADBAND = 0.2;
    }

    public static final class SwerveConfigs {
      // true para correção de aceleração
      public static final boolean accelCorrection = false;
      // constante para diminuir o input do joystick (0 < multiplicadorRotacional <= 1).
      public static final double multiplicadorRotacional = 0.5;
      // constante para diminuir o input do joystick (y).
      public static final double multiplicadorTranslacionalY = 0.5;
      // constante para diminuir o input do joystick (x).
      public static final double multiplicadorTranslacionalX = 0.5;
      // constante para diminuir o input do joystick para que o robo nao gire tanto.
      public static final double TURN_CONSTANT = 0.75;
      //constante para ajustar a aceleração do robo
      public static final double dt = 0.03;
      //constante para ajustar a rotação, sado juntamente com o turn_constant
      public static final double constantRotation = 4;
      //usada para quando há um desviao de angulo.
      public static final boolean usarCorrecaoDesvioVelocidadeAngular = true;
      //coeficiente de correção de angulo.
      public static final double coeficienteCorrecaoAngVel = 0.2;
      //correção através do pigeon.
      public static final boolean headingCorrection = false;
      //ajuda na precisão dos angulos caso não esteja totalmente preciso.
      public static final boolean cosineCompensator = false;
      //sincroniza o modulos caso não estejam sicronizado.
      public static final boolean moduleEncoderAutoSynchronize = false;
      //deadband permitada de imprecisao dos cancoders.
      public static final double moduleDeadBandEncoderValue = 2;
      
      //usado para ajustar as direções das rodas
      public static final Rotation2d graus90 = new Rotation2d(0,1);
      public static final Rotation2d graus180 = new Rotation2d(1,0);
    }

   

    public static final class alinhamento {
      
      //alinhamneto 3d do swerve.

      //set points para o lado Esquerdo
      public static final double setPointXEsquerda = -2; //vertical da limelight -- TY na leitura
      public static final double setPointYEsquerda = 2; //horizontal da limelight -- TX na leitura
      
      //set points para o lado Direito
      public static final double setPointXDireita = -1.5; //alinhamento esquerda -- TY na leitura
      public static final double setPointYDireita = -2; //Alinhamento esquerda  -- TX na leitura

      public static final double setPointRotationLE = -168;
      public static final double setPointRotationLD = 167.5;

      
      public static final boolean USAR_LIMELIGHT = true;
      
      public static final PIDController pidX = new PIDController(0.04, 0.00, 0.0000);// pid eixo horizontal
      public static final PIDController pidY = new PIDController(0.04, 0.00, 0.00);// pid eixo vertical
      public static final PIDController angulacao = new PIDController(0.062, 0.00, 0.00);
      public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.01, 0, 0);

    }

    public static enum turboStates {
      //define a marcha do robo.
      marchaBaixa(1), marchaAlta(2);
      public final double velocidade;
      
      private turboStates(double velocidade){
        this.velocidade = velocidade;
      }
    }

    public static final class AlinhamentoOdometria {
      
      //todos os graus da reef.
      public static final Rotation2d graus0 = Rotation2d.fromDegrees(0);
      public static final Rotation2d graus60 = Rotation2d.fromDegrees(60);
      public static final Rotation2d graus120 = Rotation2d.fromDegrees(120);
      public static final Rotation2d graus180 = Rotation2d.fromDegrees(180);
      public static final Rotation2d graus240 = Rotation2d.fromDegrees(240);
      public static final Rotation2d graus300 = Rotation2d.fromDegrees(300);

      public static final boolean visaoTrue = true;
      public static final boolean visaoFalse = false;

      public static final boolean utilizarVisao = false;
    }
    public static final class Tags {
      
      public static final int[] blueTag17 = {17};
      public static final int[] redTag8 = {8};

      public static final int[] blueTag18 = {18};
      public static final int[] redTag7 = {7};

      public static final int[] blueTag19 = {19};
      public static final int[] redTag6 = {6};

      public static final int[] blueTag20 = {20};
      public static final int[] redTag11 = {11};

      public static final int[] blueTag21 = {21};
      public static final int[] redTag10 = {10};

      public static final int[] blueTag22 = {22};
      public static final int[] redTag9 = {9};

      public static final int[] allTags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
    }

    public static final class tagsAutonomo {
      
      public static final int[] Frente = {18,7};
      
      public static final int[] frenteDireita = {17,8};

      public static final int[] FrenteEsquerda = {19,6};

      public static final int[] tras = {21,10};

      public static final int[] trasEsquerda = {20,11};
      
      public static final int[] trasDireita = {22,9};

      public static final int[] Alltags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
    }
    public static class ElevatorConstants {
      public static int elevatorLeftMotorID = 10;
      public static int elevatorRightMotorID = 9;
  
      public static final double PID_TOLERANCE = 0.1;
      public static final double kP = 0.045;
      public static final double kI = 0;
      public static final double kD = 0.0035;
  
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
  
      public static final double ELEVATOR_SPEED = 1;
      public static final double ELEVATOR_MANUAL_SPEED = 0.45;
    }
  
    public static class DeployerIntakeConstants {
      public static int deployerLeftMotorID = 15;
      public static int deployerRightMotorID = 16;
  
      public static int deployerSensor1 = 1;
      public static int deployerSensor2 = 0;
      public static int funnelSensor = 2;
  
      public static double intakeSpeed = 0.34;
    }
  
    public static class PivoConstants {
      public static int pivoMotorID = 11;
  
      public static final double PID_TOLERANCE = 0.1;
      public static final double kP = 0.45;
      public static final double kI = 0;
      public static final double kD = 0;
  
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
  
      public static final double PIVO_SPEED = 0.15;
    }
  
    public static class ClimberConstants {
      public static int climberMotorID = 12;
    }
  
    //States
  
    public static enum ElevatorState {
      CORAL(5), L1(44), L2(70), L3(117), L4(194);
      public final double position; 
      
      private ElevatorState(double position){
        this.position = position;
      }
    }
  
    public static enum ModeElevator {
      MANUAL(true), AUTOMATIC(false);
      public final boolean mode;
      
      private ModeElevator(Boolean mode){
        this.mode = mode;
      }
    }
  
    public static enum IntakeState {
      STOPPED(false), OPERATION(true);
      public final boolean operation;
      
      private IntakeState(Boolean operation){
        this.operation = operation;
      }
    }
  
    public static enum DeployerState {
      RESHOOTING(-0.15), STOPPED(0), SHOOTING(0.25);
      public final double speed; 
      
      private DeployerState(double speed){
        this.speed = speed;
      }
    }
  
    public static enum PivoState {
      CLOSED(0.63), OPEN(6);
      public final double position;
      
      private PivoState(double position){
        this.position = position;
      }
    }
  
    public static enum ClimberState {
      CLOSING(-1), STOPPED(0), OPENING(1);
      public final double speed;
      
      private ClimberState(double speed){
        this.speed = speed;
      }
    }

    public static class Timer {
      public static final double teleOp = 3;
      public static final double Autonomous = 2;
    }
  }
  
