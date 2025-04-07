package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeployerIntakeSystem;

public class ReShoot extends Command
{
    public DeployerIntakeSystem deployerIntakeSystem;

    public ReShoot(DeployerIntakeSystem deployerIntakeSystem)
    {
        this.deployerIntakeSystem = deployerIntakeSystem;
        addRequirements(deployerIntakeSystem);
    }

    @Override
    public void execute()
    {
        deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerState.RESHOOTING.speed);
    }
    
    @Override
    public boolean isFinished()
    {
        return deployerIntakeSystem.deployerSensor1.get();
        // return deployerIntakeSystem.funnelSensor.get();
    }

    @Override
    public void end(boolean interrupted) {
        deployerIntakeSystem.deployerLeftMotor.stopMotor();
    }
        
}
