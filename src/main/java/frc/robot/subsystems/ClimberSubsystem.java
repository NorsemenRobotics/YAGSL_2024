// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftClimber_motor = new CANSparkMax(Constants.MotorConstants.LEFT_CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightClimber_motor = new CANSparkMax(Constants.MotorConstants.RIGHT_CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    m_leftClimber_motor.restoreFactoryDefaults();
    m_leftClimber_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftClimber_motor.setSmartCurrentLimit(Constants.MotorConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    m_leftClimber_motor.burnFlash();

    m_rightClimber_motor.restoreFactoryDefaults();
    m_rightClimber_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimber_motor.setSmartCurrentLimit(Constants.MotorConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    m_rightClimber_motor.burnFlash();

    System.out.println("*** climber motor controllers configured.");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void extendClimber() {
  m_leftClimber_motor.set(-Constants.MotorConstants.CLIMBER_MOTOR_RUN_SPEED);
  m_rightClimber_motor.set(Constants.MotorConstants.CLIMBER_MOTOR_RUN_SPEED);  // Inverted
}

public void retractClimber() {
  m_leftClimber_motor.set(Constants.MotorConstants.CLIMBER_MOTOR_RUN_SPEED);  // Inverted
  m_rightClimber_motor.set(-Constants.MotorConstants.CLIMBER_MOTOR_RUN_SPEED);  
}

public void stopClimber() {
  m_leftClimber_motor.set(0); 
  m_rightClimber_motor.set(0);

}

}
