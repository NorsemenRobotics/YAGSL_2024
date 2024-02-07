// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TestMotor extends SubsystemBase  {
  
    // create new m_test_motor
    private final CANSparkMax m_test_motor = new CANSparkMax(Constants.TestMotor.TEST_MOTOR_CAN_ID, MotorType.kBrushless);

    CommandJoystick shooterController = new CommandJoystick(Constants.OperatorConstants.SHOOTER_USB_PORT);

    private double joystickTestMotorControlInput;
    
    //private double counter;
    //private double counterPer;
    // I am not sure what this TestMotor() {} does (it's empty...?), but it is present in the wpilib documentation so I duplicated it.
    // Maybe it is there to placehold for a call to require this whole class in a command?
    public TestMotor(double someInput) {
      
    } 
    public TestMotor() {
      //counter = 0;
      m_test_motor.restoreFactoryDefaults();
      m_test_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    } 

    @Override
    public void periodic()
    {
      //counterPer = counterPer + 1;
      //SmartDashboard.putNumber("Test Motor Periodic", counterPer);
    }

    public void runMotor() {
      //counter = counter + 1;
      //SmartDashboard.putNumber("Test Motor", counter);
      joystickTestMotorControlInput = shooterController.getRawAxis(3);
      m_test_motor.set(joystickTestMotorControlInput);
      SmartDashboard.putNumber("Test Motor Speed", joystickTestMotorControlInput);
    }

    public void stopMotor(){
      m_test_motor.set(0);
    }
}
