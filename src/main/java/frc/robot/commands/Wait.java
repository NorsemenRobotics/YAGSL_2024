// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// This command should take a double in ms and wait to return complete

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


public class Wait extends Command {
  /** Creates a new Wait. */
  
  protected long timeInMillis;
  protected long endTime; // variable to store end time
  protected double time;  // variable to store current time

  public Wait(long timeInMillis) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long startTime = System.currentTimeMillis();
    endTime = startTime + timeInMillis;
    System.out.println("*** TIME SET: " + timeInMillis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endTime;  //should return true when timeinMillis has expired
  }
}
