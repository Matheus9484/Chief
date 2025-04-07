// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class GetSensor extends InstantCommand {
  boolean finished = false;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public double getAnalogSensorDistance(AnalogInput sensor) {
      double voltage = sensor.getVoltage();
      double distanceCm = 27.86 / (voltage - 0.42);
      return distanceCm;
    }

    public boolean getDigitalSensor(DigitalInput sensor){
      return sensor.get();
    }

    public void inteikar(){
      if(getDigitalSensor(RobotContainer.deployerIntakeSystem.funnelSensor)){
        finished = true;
      }
    }
    
    @Override
    public boolean isFinished()
    {
        return finished;
    }
  
}
