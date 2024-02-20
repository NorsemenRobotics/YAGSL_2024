// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ServoSubsystem extends SubsystemBase {

  private final Servo m_deflectorServo = new Servo(Constants.ServoConstants.DEFLECTOR_SERVO_PORT);
  /** Creates a new ServoSubsystem. */
  public ServoSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void servoDeflectorOn() {
    m_deflectorServo.setAngle(Constants.ServoConstants.DEFLECTOR_SERVO_ON_ANGLE);
  }

  public void servoDeflectorOff() {
    m_deflectorServo.setAngle(Constants.ServoConstants.DEFLECTOR_SERVO_OFF_ANGLE);
  }

}
