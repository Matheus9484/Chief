// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeState;
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.Controle;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;
import frc.robot.Constants.Tags;
import frc.robot.Constants.Timer;
import frc.robot.Constants.turboStates;
import frc.robot.commands.AutoCommands.SwerveLadoEsquerdaAuto;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetActions;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.commands.OdometriaAlinhamento.AlinhamentoOdometria;
import frc.robot.commands.Reef.Alinhamento3dDireita;
import frc.robot.commands.Reef.Alinhamento3dEsquerda;
import frc.robot.commands.Reef.AlinhamentoMeio;
import frc.robot.commands.Subsystem.Intake;
import frc.robot.commands.Subsystem.ReShoot;
import frc.robot.commands.Subsystem.ReShootD;
import frc.robot.commands.Subsystem.Shoot;
import frc.robot.commands.Subsystem.ShootD;
import frc.robot.commands.Subsystem.moveElevatorCommand;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoDireita;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoEsquerda;
import frc.robot.commands.swervedrive.drivebase.SwerveLadoFrente;
import frc.robot.commands.swervedrive.drivebase.Teleop;
import frc.robot.subsystems.AlgaeSystem;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.DeployerIntakeSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.PivoSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  // Aqui iniciamos o swerve
  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private turboStates turbo = turboStates.marchaBaixa;
  boolean modoAlinhamento;

  boolean alianca = true;// true para vermelha, false para azul
  public final static ElevatorSystem elevatorSystem = new ElevatorSystem();
  public static final DeployerIntakeSystem deployerIntakeSystem = new DeployerIntakeSystem();
  public static final PivoSystem pivoSystem = new PivoSystem();
  public static final ClimberSystem climberSystem = new ClimberSystem();
  public static final AlgaeSystem algaeSystem = new AlgaeSystem();

  public static final Led led = new Led();
  
  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<Integer> pipelineChooser = new SendableChooser<>();
  public moveElevatorCommand moveElevatorCommand;
  GetSensor getSensor;

  //Controle piloto
  CommandXboxController controleXbox = new CommandXboxController(Controle.xboxControle);

  CommandXboxController buttonJoy = new CommandXboxController(Controle.buttonBox);
  
  CommandXboxController joymanual = new CommandXboxController(Controle.joyManual);

  public RobotContainer() {
    // Definimos o comando padrão como a tração
    swerve.setDefaultCommand(new Teleop(swerve,
      () -> -MathUtil.applyDeadband(controleXbox.getLeftY() * turbo.velocidade, Constants.Controle.DEADBAND),
      () -> -MathUtil.applyDeadband(controleXbox.getLeftX() * turbo.velocidade, Constants.Controle.DEADBAND) ,
      () -> -MathUtil.applyDeadband(controleXbox.getRightX() * turbo.velocidade / 1.7, Constants.Controle.DEADBAND)));

    
    //  NamedCommands.registerCommand("Stop", 
    //     new InstantCommand(() -> swerve.stop(), swerve)
    // );
    //  NamedCommands.registerCommand("Encoder", 
    //     new InstantCommand(() -> moveElevatorCommand.ConferirEnconder(), elevatorSystem)
    // );

    
    // NamedCommands.registerCommand("L1", 
    // new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L2", 
    // new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L3", 
    //  new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("L4", 
    //  new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("Repouso", 
    //  new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    // NamedCommands.registerCommand("Pontuar", 
    //  new Shoot(deployerIntakeSystem));
    // NamedCommands.registerCommand("Recolher",
    //  new SequentialCommandGroup(new Intake(deployerIntakeSystem), new ReShoot(deployerIntakeSystem)));
    
    
      NamedCommands.registerCommand("AlinharD17", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.frenteDireita, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE17", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.frenteDireita, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE18", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.Frente, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD18", 
      new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.Frente, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD19", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.FrenteEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE19", 
      new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.FrenteEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE20", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.trasEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD20", 
      new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.trasEsquerda, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharD21", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.tras, Timer.Autonomous)
      );
    
      NamedCommands.registerCommand("AlinharE21", 
       new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.tras, Timer.Autonomous)
      );
      NamedCommands.registerCommand("AlinharE22", 
    new Alinhamento3dDireita(swerve, controleXbox, Constants.tagsAutonomo.trasDireita, Timer.Autonomous)
);
 
     
      NamedCommands.registerCommand("AlinharD22", 
       new Alinhamento3dEsquerda(swerve, controleXbox, Constants.tagsAutonomo.trasDireita, Timer.Autonomous)
      );

      NamedCommands.registerCommand("L1", 
      new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
      NamedCommands.registerCommand("L2", 
      new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
      NamedCommands.registerCommand("L3", 
       new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
      NamedCommands.registerCommand("L4", 
       new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
      NamedCommands.registerCommand("Repouso", 
       new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
      NamedCommands.registerCommand("Pontuar", 
       new Shoot(deployerIntakeSystem));
      NamedCommands.registerCommand("Recolher",
       new SequentialCommandGroup(new Intake(deployerIntakeSystem)));
      
      
      autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomo", autoChooser);
    




    if(!Robot.isReal()){
      controleXbox.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    // Configure the trigger bQindings
    configureBindings();
    configurePipelineSelector();
    defaultcommands();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    

     controleXbox.y().whileTrue(new SwerveLadoFrente(controleXbox, swerve));
     controleXbox.x().whileTrue(new SwerveLadoDireita(controleXbox, swerve));    
     controleXbox.b().whileTrue(new SwerveLadoEsquerda(controleXbox, swerve));
    //controleXbox.a().whileTrue(new SwerveAutoAlinhamentoCompleto(swerve, controleXbox));

    controleXbox.leftBumper().onTrue(new InstantCommand(() -> turbo = turboStates.marchaBaixa));
    controleXbox.rightBumper().onTrue(new InstantCommand(() -> turbo = turboStates.marchaAlta));

    controleXbox.start().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    
    controleXbox.povUp().onTrue(new InstantCommand(() -> swerve.visao = false));
    controleXbox.povDown().onTrue(new InstantCommand(() -> swerve.visao = true));

    //controleXbox.y().onTrue(new InstantCommand(() -> modoAlinhamento = false));  
    //controleXbox.a().onTrue(new InstantCommand(() -> modoAlinhamento = true));

    joymanual.rightTrigger().whileTrue(new SetMechanismState(AlgaeState.GET)).onFalse(new SetMechanismState(AlgaeState.STOPPED));
    joymanual.leftTrigger().whileTrue(new SetMechanismState(AlgaeState.SHOOT)).onFalse(new SetMechanismState(AlgaeState.STOPPED));
    // joymanual.leftTrigger().onTrue(new Intake(deployerIntakeSystem));

    joymanual.button(5).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joymanual.button(1).onTrue(new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joymanual.button(2).onTrue(new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joymanual.button(4).onTrue(new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joymanual.button(3).onTrue(new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));

    joymanual.povUp().onTrue(new SetMechanismState(PivoState.OPEN));
    joymanual.povDown().onTrue(new SetMechanismState(PivoState.CLOSED));

    joymanual.button(6).whileTrue(new ReShootD(deployerIntakeSystem));

    joymanual.button(8).onTrue(new SetActions().ClimbPosition());

    controleXbox.rightTrigger().onTrue(new SetMechanismState(ClimberState.OPENING)).onFalse(new SetMechanismState(ClimberState.STOPPED));
    controleXbox.leftTrigger().onTrue(new SetMechanismState(ClimberState.CLOSING)).onFalse(new SetMechanismState(ClimberState.STOPPED));

    buttonJoy.button(15).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    buttonJoy.button(13).onTrue(new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    buttonJoy.button(11).onTrue(new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    buttonJoy.button(9).onTrue(new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    buttonJoy.button(6).onTrue(new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));

    // buttonJoy.button(4).onTrue(new Intake(deployerIntakeSystem));
    buttonJoy.button(4).onTrue(new SequentialCommandGroup(new Intake(deployerIntakeSystem), new ReShoot(deployerIntakeSystem)));
    buttonJoy.button(1).whileTrue(new ShootD(deployerIntakeSystem));
    buttonJoy.button(7).onTrue(new SetActions().ClimbPosition());
    ButtonBoxPP();

    controleXbox.y().onTrue(new InstantCommand(() -> {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Tags.allTags);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Tags.allTags);
    }));
  }
  
  // funcao que retorna o autonomous
  
  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

   private void configurePipelineSelector() {
        // Adiciona opções de pipelines
        pipelineChooser.setDefaultOption("Pipeline 0", 0);
        pipelineChooser.addOption("Pipeline 1", 1);
        pipelineChooser.addOption("Pipeline 2", 2);
        pipelineChooser.addOption("Pipeline 3", 3);
        pipelineChooser.addOption("Pipeline 4", 4);
        pipelineChooser.addOption("Pipeline 5", 5);
        pipelineChooser.addOption("Pipeline 6", 6);

        // Envia o chooser para a SmartDashboard
        SmartDashboard.putData("Pipeline Selector", pipelineChooser);
    }

    public int getSelectedPipeline() {
      return pipelineChooser.getSelected();
  }

  public void ButtonBoxPP(){
    
    
    //Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    //boolean isRedAlliance = (alliance == Alliance.Red);
    boolean isRedAlliance = alianca;

    System.out.println("Estamos na aliança " + (isRedAlliance ? "vermelha" : "azul") + "!");
    // Frente (Direita)
    buttonJoy.button(16).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 7 : 18, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0 , 0, true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente (Esquerda)
    buttonJoy.button(14).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 7 : 18, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0, true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag7 : Constants.Tags.blueTag18, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Frente Direita (Direita)
    buttonJoy.button(12).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 8 : 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente Direita (Esquerda)
    buttonJoy.button(10).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 8 : 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Frente Esquerda (Direita)
    buttonJoy.button(18).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 6 : 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Frente Esquerda (Esquerda)
    buttonJoy.button(17).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 6 : 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.teleOp),
        () -> modoAlinhamento
    ));


    //Tras Direita (Direita)
    buttonJoy.button(8).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Direita (Esquerda)
    buttonJoy.button(5).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Esquerda (Direita)
    buttonJoy.button(20).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras Esquerda (Esquerda)
    buttonJoy.button(19).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.teleOp),
        () -> modoAlinhamento
    ));
    
    //Tras (Direita)
    buttonJoy.button(2).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp)),
        new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp),
        () -> modoAlinhamento
    ));

    //Tras (Esquerda)
    buttonJoy.button(21).onTrue(Commands.either(
        new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0, 0,  true, false)
            .andThen(new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp)),
        new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.teleOp),
        () -> modoAlinhamento
    ));
}
    public Command Autonomo() {
        // Aqui retornamos o comando que está no selecionador
        return autoChooser.getSelected();   
      }

    private void defaultcommands(){
    elevatorSystem.setDefaultCommand(new moveElevatorCommand(joymanual, elevatorSystem));
  }
  public Command Direita(){ 
    boolean isRedAlliance = alianca; // true para vermelho, e false para azul


   return new SequentialCommandGroup(

   
        //alinhar 
     //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 9 : 22, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0,0,true, false ),
     
      new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.Autonomous),
      // pontuar
      
      new Shoot(deployerIntakeSystem),
      //recolher
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new WaitCommand(0.4),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 2 : 12, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.46, 1.5, true, false),
       new Intake(deployerIntakeSystem),
       new ReShoot(deployerIntakeSystem),
       
        
       // alinhar
      // new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ?  8: 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, true ),
      //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.Autonomous),
      //pontuar
      
      
      new Shoot(deployerIntakeSystem),
      
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new WaitCommand(0.4),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 2 : 12, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.46, 1.5, true, false),
      new Intake(deployerIntakeSystem),
      new ReShoot(deployerIntakeSystem),
      // new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ?  8: 17, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0 , false, true),
      //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag8 : Constants.Tags.blueTag17, Timer.Autonomous),
       new Shoot(deployerIntakeSystem)
      );
  }
  public Command Esquerda(){
    boolean isRedAlliance = alianca; // true para vermelho, e false para azul

   return new SequentialCommandGroup(
    //alinhar 
  //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
  new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0,0,true, false),
  new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag11 : Constants.Tags.blueTag20, Timer.Autonomous),
  // pontuar
  
  new Shoot(deployerIntakeSystem),
  //recolher
  //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
  new WaitCommand(0.4),
  new AlinhamentoOdometria(swerve, isRedAlliance ? 1 : 13, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.55, 1.5, true, false),
  new Intake(deployerIntakeSystem),
  new ReShoot(deployerIntakeSystem),
  
 
  
  
  // alinhar
  //new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
  new AlinhamentoOdometria(swerve, isRedAlliance ?  6: 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, true),
  //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
 new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.Autonomous),
  //pontuar
  
  new Shoot(deployerIntakeSystem),
  //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
  new WaitCommand(0.4),
  new AlinhamentoOdometria(swerve, isRedAlliance ? 1 : 13, buttonJoy, Constants.AlinhamentoOdometria.graus0, 0.55, 1.5, true, false),
  new Intake(deployerIntakeSystem),
  new ReShoot(deployerIntakeSystem),
  //new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
  new AlinhamentoOdometria(swerve, isRedAlliance ?  6: 19, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, true),
  //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
 new Alinhamento3dDireita(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag6 : Constants.Tags.blueTag19, Timer.Autonomous),
  //pontuar
  
  new Shoot(deployerIntakeSystem)
  
  
  

  );
  }
  public Command Meio(){
    boolean isRedAlliance = alianca; // true para vermelho, e false para azul

    return new SequentialCommandGroup(
        //alinhar 
        
      //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, true, false),
      new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag10 : Constants.Tags.blueTag21, Timer.Autonomous),
      // pontuar
      
      new Shoot(deployerIntakeSystem)
      //recolher
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC))
    

      );
  }

  public Command MeioAlga(){
    boolean isRedAlliance = alianca;

    return new SequentialCommandGroup(
        //alinhar 
        
      //new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 10 : 21, buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, true, false),
      new Alinhamento3dEsquerda(swerve, buttonJoy, isRedAlliance ? Constants.Tags.redTag9 : Constants.Tags.blueTag22, Timer.Autonomous),
      // pontuar
      
      new Shoot(deployerIntakeSystem),

      new SwerveLadoEsquerdaAuto(controleXbox, swerve, 1),
      
      new AlinhamentoOdometria(swerve, isRedAlliance ? 5 : 14, buttonJoy, Constants.AlinhamentoOdometria.graus180, 0.7, 0, false, false),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 11 : 20 , buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, false),
      new AlinhamentoMeio(),
      new AlinhamentoOdometria(swerve, isRedAlliance ? 5 : 14 , buttonJoy, Constants.AlinhamentoOdometria.graus180, 1.0, 0, false, false)
      //recolher
      //new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC))
    

      );
      
  }
}