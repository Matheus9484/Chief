package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeployerIntakeSystem;

public class Intake extends Command
{
    public DeployerIntakeSystem deployerIntakeSystem;

    public Intake(DeployerIntakeSystem deployerIntakeSystem)
    {
        this.deployerIntakeSystem = deployerIntakeSystem;
        addRequirements(deployerIntakeSystem);
    }

    @Override
    public void execute()
    {
        deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerIntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished()
    {
        // return !deployerIntakeSystem.deployerSensor2.get();

        // return !deployerIntakeSystem.deployerSensor2.get()
        // public static double intakeSpeed = 0.8

        // return !deployerIntakeSystem.deployerSensor2.get() && !deployerIntakeSystem.deployerSensor1.get();
        return !deployerIntakeSystem.deployerSensor2.get();
    }
    
    @Override
    public void end(boolean interrupted) {
        Timer.delay(0.04);
        deployerIntakeSystem.deployerLeftMotor.stopMotor();
    }
        
}
