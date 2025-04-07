// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Tags;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.subsystems.DeployerIntakeSystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
    private DataLog log;
    private DoubleLogEntry sensorLog;
    GetSensor sensor = new GetSensor();
   
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Tags.allTags);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Tags.allTags);
    PathfindingCommand.warmupCommand().schedule();
    m_robotContainer = new RobotContainer();
    DataLogManager.start(); // Inicia o gerenciador de logs
    DataLog log = DataLogManager.getLog();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    SmartDashboard.putString("Auto", "none");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    
    boolean sensorValue = sensor.getDigitalSensor(m_robotContainer.deployerIntakeSystem.deployerSensor1); // Lê o valor do sensor
    boolean sensorValue2 = sensor.getDigitalSensor(m_robotContainer.deployerIntakeSystem.deployerSensor2); // Lê o valor do sensor
    double elevator = m_robotContainer.elevatorSystem.getElevatorEncoderPosition(); // Lê o valor do sensor
        // sensorLog.append(sensorValue ? 1.0 : 0.0);
        // sensorLog.append(sensorValue2 ? 1.0 : 0.0);
        // sensorLog.append(elevator);
         // Salva o valor no logp
    // SmartDashboard.putBoolean("Sensor1", sensorValue);
    // SmartDashboard.putBoolean("Sensor2", sensorValue2);

      DriverStation.reportWarning("sensor1 " + sensorValue, false); // Mostra no dashboard
      DriverStation.reportWarning("sensor2" + sensorValue2, false); // Mostra no dashboard
      DriverStation.reportWarning("elevator" + elevator, false); // Mostra no dashboard
    int selectedPipeline = m_robotContainer.getSelectedPipeline();
    LimelightHelpers.setPipelineIndex("limelight", selectedPipeline);
    LimelightHelpers.setPipelineIndex("limelight-direita", selectedPipeline);
    SmartDashboard.putNumber("Pipeline Atual", selectedPipeline);

    // SmartDashboard.putData("Camera", CameraServer.getServer());

    

     m_robotContainer.swerve.updateVisionOdometry(); //funcao que atualiza a odometria com a visao.
    CommandScheduler.getInstance().run();
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    String autoMode = SmartDashboard.getString("Auto", "none");
if (autoMode.equalsIgnoreCase("autodireita")){
  System.out.println("Funcionou");
  m_autonomousCommand = m_robotContainer.Direita();
}else if(autoMode.equalsIgnoreCase("Autoesquerda")){
  m_autonomousCommand = m_robotContainer.Esquerda();
}else if(autoMode.equalsIgnoreCase("Automeio")){
  m_autonomousCommand = m_robotContainer.Meio();
}
   
    
    
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Tags.allTags);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Tags.allTags);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //m_robotContainer.swerve.visao = Constants.AlinhamentoOdometria.visaoTrue;
  }

  @Override
  public void teleopInit() {


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    

    
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-direita", Tags.allTags);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Tags.allTags);
    
    m_robotContainer.swerve.visao = Constants.AlinhamentoOdometria.visaoTrue;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    // Record both DS control and joystick data
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void close() {
  }
}