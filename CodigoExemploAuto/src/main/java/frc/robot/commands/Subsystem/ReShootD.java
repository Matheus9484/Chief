package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeployerIntakeSystem;

public class ReShootD extends Command
{
    public DeployerIntakeSystem deployerIntakeSystem;

    public ReShootD(DeployerIntakeSystem deployerIntakeSystem)
    {
        this.deployerIntakeSystem = deployerIntakeSystem;
        addRequirements(deployerIntakeSystem);
    }

    @Override
    public void execute()
    {
        deployerIntakeSystem.deployerLeftMotor.set(-0.35);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
        // return deployerIntakeSystem.funnelSensor.get();
    }

    @Override
    public void end(boolean interrupted) {
        deployerIntakeSystem.deployerLeftMotor.stopMotor();
    }
        
}
