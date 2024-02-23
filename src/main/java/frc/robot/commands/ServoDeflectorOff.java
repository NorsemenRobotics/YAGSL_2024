// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSubsystem;

public class ServoDeflectorOff extends Command {

  private final ServoSubsystem m_servo_motor;

  public ServoDeflectorOff(ServoSubsystem subsystem) {
    m_servo_motor = subsystem;
    addRequirements(m_servo_motor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       m_servo_motor.servoDeflectorOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
