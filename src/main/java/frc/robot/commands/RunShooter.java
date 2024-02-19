// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//FIXME: This command runs the shooter motors; note that the speed is set in the subsystem.  Also, the subsystem overrides the speed with joystick.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {

  private final ShooterSubsystem m_ShooterMotors;
  


  /** Creates a new RunShooter. */
  public RunShooter(ShooterSubsystem subsystem) {
    m_ShooterMotors = subsystem;
      addRequirements(m_ShooterMotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterMotors.runShooterMotors(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterMotors.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
