package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeployerIntakeSystem;

public class Shoot extends Command
{
    public DeployerIntakeSystem deployerIntakeSystem;

    public Shoot(DeployerIntakeSystem deployerIntakeSystem)
    {
        this.deployerIntakeSystem = deployerIntakeSystem;
        addRequirements(deployerIntakeSystem);
    }

    @Override
    public void execute()
    {
        deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerState.SHOOTING.speed);
    }
    
    @Override
    public boolean isFinished()
    {
        return deployerIntakeSystem.deployerSensor2.get();
    }
    
    @Override
    public void end(boolean interrupted) {
        deployerIntakeSystem.deployerLeftMotor.stopMotor();
    }
        
}
