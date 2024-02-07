// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotor;

public class RunTestMotor extends Command {
  
  private final TestMotor m_TestMotor;
  // private double counter;
  // private double motorSpeed;
    public RunTestMotor(TestMotor subsystem) {
        m_TestMotor = subsystem;
       // motorSpeed = speed;
        addRequirements(m_TestMotor);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
     // counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    // counter = counter + 1;
    //  SmartDashboard.putNumber("Test Motor Command", counter);
      m_TestMotor.runMotor();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      //added to turn motor off at end of command
      m_TestMotor.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
