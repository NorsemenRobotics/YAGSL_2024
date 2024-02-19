// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This command is to set shooter motors to the speaker shooting speed.  It does not shut the motors off.  Use StopShooter to do that.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class ShootSpeaker extends Command {
  /** Creates a new SetShooter. */
  private final ShooterSubsystem m_ShooterMotors;

  protected long timeInMillis = Constants.MotorConstants.SHOOTER_SPOOLUP_WAIT_TIME_MS;  // total time to run in ms
  protected long endTime; // variable to store end time
  protected double time;  // variable to store current time


  public ShootSpeaker(ShooterSubsystem subsystem) {   
    m_ShooterMotors = subsystem;
    addRequirements(m_ShooterMotors);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long startTime = System.currentTimeMillis();
    endTime = startTime + timeInMillis;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterMotors.runShooterMotors(Constants.MotorConstants.SPEAKER_SHOOT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endTime;  //should return true when timeinMillis has expired
  }
}
