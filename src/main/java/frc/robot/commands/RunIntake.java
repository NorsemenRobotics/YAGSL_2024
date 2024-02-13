// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
  
  private final IntakeSubsystem m_IntakeMotors;

    public RunIntake(IntakeSubsystem subsystem) {
        m_IntakeMotors = subsystem;
           addRequirements(m_IntakeMotors);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_IntakeMotors.runIntakeMotors();
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
