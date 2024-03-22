// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteSensor extends SubsystemBase {

  //create new noteSensor to detect note in magazine
  DigitalInput noteSensor = new DigitalInput(0);
  //private boolean notePresent = false;


  /** Creates a new NoteSensor. */
  public NoteSensor() {
    System.out.println("*** note sensor configured.");

  }

  @Override
  public void periodic() {

    // Gets the value of the digital input.  Returns true if the circuit is open.
    // notePresent = noteSensor.get();
      SmartDashboard.putBoolean("Note Present:", !noteSensor.get());

    // This method will be called once per scheduler run
  }
}
