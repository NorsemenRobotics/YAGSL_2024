// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This command is to run the intake using vX to drive over notes without picking them up.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class IdleIntake extends Command {

  private final IntakeSubsystem m_IntakeMotors;

  /** Creates a new IdleIntake. */
  public IdleIntake(IntakeSubsystem subsystem) {
        m_IntakeMotors = subsystem;
           addRequirements(m_IntakeMotors);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeMotors.idleIntakeMotors(Constants.MotorConstants.INTAKE_IDLE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeMotors.stopIntakeMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
