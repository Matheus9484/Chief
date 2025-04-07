// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;

public class Led extends SubsystemBase {
  public PWM RED;
  public PWM GREEN;
  public PWM BLUE;

  public boolean StateR;
  public boolean StateG;
  public boolean StateB;

  public Led() {
    RED = new PWM(2);
    GREEN = new PWM(0);
    BLUE = new PWM(1);
  }

  public void periodic() {
    if(StateR){
      RED.setAlwaysHighMode();
      Timer.delay(0.5);
      RED.setDisabled();
    } 
    
    if(StateG){
      GREEN.setAlwaysHighMode();
      Timer.delay(0.5);
      GREEN.setDisabled();
    } 
    
    if(StateB){
      BLUE.setAlwaysHighMode();
      Timer.delay(0.5);
      BLUE.setDisabled();
    }
  }

  public void SetStateLedRed(Boolean State){
    this.StateR = State;
  }

  public void SetStateLedGreen(Boolean State){
    this.StateG = State;
  }

  public void SetStateLedBlue(Boolean State){
    this.StateB = State;
  }
}
