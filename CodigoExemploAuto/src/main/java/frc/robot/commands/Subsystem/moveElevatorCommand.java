package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ModeElevator;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.subsystems.ElevatorSystem;

public class moveElevatorCommand extends Command
{
    private ElevatorSystem elevatorSubsystem;
    private CommandXboxController joystick2;
    boolean finished = false;

    public moveElevatorCommand(CommandXboxController joystick2, ElevatorSystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick2 = joystick2;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute()
    {
        if(Math.abs(joystick2.getLeftY()) > 0.1){
            elevatorSubsystem.moveElevatorManual(joystick2.getLeftY());
            CommandScheduler.getInstance().schedule(new SetMechanismState(ModeElevator.MANUAL));
        } else{
            elevatorSubsystem.moveElevatorManual(0);
        }
        
    }
    // esse método confere se o elevador está no estágio correto (usado no autonomous)
    // public void ConferirEnconder(){
    //     if(elevatorSubsystem.getElevatorEncoderPosition() >= 315 && elevatorSubsystem.getElevatorEncoderPosition() <= 330 && elevatorSubsystem.stateElevator == ElevatorState.L4){
    //         finished = true;
    //     }else if (elevatorSubsystem.getElevatorEncoderPosition() >= 180 &&  elevatorSubsystem.getElevatorEncoderPosition() <= 205 && elevatorSubsystem.stateElevator == ElevatorState.L3){
    //         finished = true;
    //     }else if (elevatorSubsystem.getElevatorEncoderPosition() >= 90 && elevatorSubsystem.getElevatorEncoderPosition() <= 150 && elevatorSubsystem.stateElevator == ElevatorState.L2){
    //         finished = true;
    //     }else if (elevatorSubsystem.getElevatorEncoderPosition() >= 60 && elevatorSubsystem.getElevatorEncoderPosition() <= 89  && elevatorSubsystem.stateElevator == ElevatorState.L1){
    //         finished = true;
    //     }else if (elevatorSubsystem.getElevatorEncoderPosition() >= 0 && elevatorSubsystem.getElevatorEncoderPosition() <= 12 && elevatorSubsystem.stateElevator == ElevatorState.CORAL){
    //         finished = true;
    //     }
    // }
  
    @Override
    public boolean isFinished()
    {
        return finished;
    }
        
}
