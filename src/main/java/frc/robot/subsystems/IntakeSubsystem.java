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

public class IntakeSubsystem extends SubsystemBase  {
  
    // create new intake motors
    private final CANSparkMax m_frontIntake_motor = new CANSparkMax(Constants.MotorConstants.FRONT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_backIntake_motor = new CANSparkMax(Constants.MotorConstants.BACK_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    CommandJoystick shooterController = new CommandJoystick(Constants.OperatorConstants.SHOOTER_USB_PORT);

    private double joystickIntakeMotorControlInput;
    
  
    // I am not sure what this TestMotor() {} does (it's empty...?), but it is present in the wpilib documentation so I duplicated it.
    // Maybe it is there to placehold for a call to require this whole class in a command?

    //public IntakeSubsystem(double someInput) {
      
    //} 
    public IntakeSubsystem() {

      m_frontIntake_motor.restoreFactoryDefaults();
      m_frontIntake_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_frontIntake_motor.setSmartCurrentLimit(Constants.MotorConstants.INTAKE_MOTOR_CURRENT_LIMIT);
      m_frontIntake_motor.burnFlash();

      m_backIntake_motor.restoreFactoryDefaults();
      m_backIntake_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_backIntake_motor.setSmartCurrentLimit(Constants.MotorConstants.INTAKE_MOTOR_CURRENT_LIMIT);
      m_backIntake_motor.burnFlash();

      System.out.println("*** intake motor controllers configured.");
    } 

    @Override
    public void periodic()
    {
    }

    public void runIntakeMotors() {
      joystickIntakeMotorControlInput = shooterController.getRawAxis(3);
      m_frontIntake_motor.set(joystickIntakeMotorControlInput);
      m_backIntake_motor.set(-joystickIntakeMotorControlInput);
      SmartDashboard.putNumber("Test Motor Speed", joystickIntakeMotorControlInput);
    }

    public void stopIntakeMotors(){
      m_frontIntake_motor.set(0);
      m_backIntake_motor.set(0);
    }
}
