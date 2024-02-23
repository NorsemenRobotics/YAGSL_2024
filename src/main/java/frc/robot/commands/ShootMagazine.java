// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;

public class ShootMagazine extends Command {

  private final MagazineSubsystem m_MagazineMotor;
  /** Creates a new StageMagazine. */

  protected long timeInMillis = Constants.MotorConstants.MAGAZINE_SHOOT_RUN_TIME_MS;  // total time to run in ms
  protected long endTime; // variable to store end time
  protected double time;  // variable to store current time

  public ShootMagazine(MagazineSubsystem subsystem) {
    m_MagazineMotor = subsystem;
    //this.time = timeInMillis;
    addRequirements(m_MagazineMotor);
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

    //TODO:  Here the value appears to work sending to command
    m_MagazineMotor.setMagazineMotor(Constants.MotorConstants.MAGAZINE_FEED_TO_SHOOTER_VOLTS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_MagazineMotor.setMagazineMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endTime;  //should return true when timeinMillis has expired
  }
}
