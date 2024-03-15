// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
//import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
//TODO: Define robot mass
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
    public static final double AUTON_MAX_MODULE_SPEED_MS = 4.5;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    
    // Casey-proof robot
    public static final double DRIVER_FACTOR = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double LEFT_Z_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    //public static final double TURN_CONSTANT = 0.75;
    public static final double TURN_FACTOR = 1; // was 0.825

    public static final int DRIVER_USB_PORT = 0;
    public static final int SHOOTER_USB_PORT = 1;
    public static final int DRIVER_ROTATION_AXIS = 4;
  }

  public static class MotorConstants
  {
    public static final int FRONT_INTAKE_MOTOR_CAN_ID = 31;
    public static final int BACK_INTAKE_MOTOR_CAN_ID = 32;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 20;

    public static final int LEFT_SHOOTER_MOTOR_CAN_ID = 33;
    public static final int RIGHT_SHOOTER_MOTOR_CAN_ID = 34;
    public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 20;

    public static final int MAGAZINE_MOTOR_CAN_ID = 35;
    public static final int MAGAZINE_MOTOR_CURRENT_LIMIT = 20;

    public static final int LEFT_CLIMBER_MOTOR_CAN_ID = 36;
    public static final int RIGHT_CLIMBER_MOTOR_CAN_ID = 37;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 30;
    public static final double CLIMBER_MOTOR_RUN_SPEED = 0.6;

    public static final double MAGAZINE_INTAKE_VOLTS = 9.5;          
    public static final double MAGAZINE_STAGE_VOLTS = -2.45;           //back off note speed
    public static final long MAGAZINE_STAGE_RUN_TIME_MS = 150;        //time of backing off note
    public static final long MAGAZINE_SHOOT_RUN_TIME_MS = 1000;       //time of shooting note
    public static final double MAGAZINE_FEED_TO_SHOOTER_VOLTS = 9.5; //speed to feed note to shooter


    public static final double SPEAKER_SHOOT_VELOCITY = 10000; // 10000 works at speaker base
  
    public static final double AMP_SHOOT_VELOCITY = 4300; // was 4400 -- 4300 works

    public static final double INTAKE_INTAKE_VOLTS = 10.5;            //pickup off the floor
    public static final double INTAKE_PUKE_VOLTS = 9.3; 
    public static final double INTAKE_IDLE_SPEED = 0.2;

    public static final long SHOOTER_SPOOLUP_WAIT_TIME_MS = 500; // time to wait for shooter wheel spoolup.  was 1000

  }

public static class ServoConstants
{
  public static final int DEFLECTOR_SERVO_PORT = 9;
  public static final double DEFLECTOR_SERVO_ON_ANGLE = 169; // increasing degrees bring deflector more toward shooter; 168 was 33 degrees
  public static final double DEFLECTOR_SERVO_OFF_ANGLE = 0;

}

public static class ShooterPIDConstants
{
  public static final double SHOOTER_PID_kP = 6e-5; 
  public static final double SHOOTER_PID_kI = 0;
  public static final double SHOOTER_PID_kD = 0; 
  public static final double SHOOTER_PID_kIz = 0; 
  public static final double SHOOTER_PID_kFF = 0.000015; 
  public static final double SHOOTER_PID_kMaxOutput = 1; 
  public static final double SHOOTER_PID_kMinOutput = -1;
  public static final double SHOOTER_PID_maxRPM = 5700;
}

}
