// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class StopShooter extends Command {
  /** Creates a new SetShooter. */
  private final ShooterSubsystem m_ShooterMotors;


  public StopShooter(ShooterSubsystem subsystem) {   
    m_ShooterMotors = subsystem;
    addRequirements(m_ShooterMotors);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_ShooterMotors.stopShooterMotorsRPM();
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
  //Added 'true' to force command to end after one iteration?
    return true; 
  }
}