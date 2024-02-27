/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to
 * control the position of the motor, and includes a max velocity and max
 * acceleration parameter to ensure the motor moves in a smooth and predictable
 * way. This is done by generating a motion profile on the fly in SPARK MAX and
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only
 * two steps required to configure this mode:
 * 1) Tune a velocity PID loop for the mechanism
 * 2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the
 * velocity
 * PID, is to graph the inputs and outputs to understand exactly what is
 * happening.
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 * 1) The velocity of the mechanism (‘Process variable’)
 * 2) The commanded velocity value (‘Setpoint’)
 * 3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure
 * to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Robot extends LoggedRobot {
  private static final int deviceID = 15;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  public double kP, kD, kV, maxVel, maxAcc;
  public double currentPosition;
  public double setPoint;

  public double p;
  public double d;
  public double v;
  public double maxV;
  public double maxA;

  // Creates a ProfiledPIDController
  // Max velocity is 5 meters per second
  // Max acceleration is 10 meters per second
  ProfiledPIDController profiledPIDController;
  ArmFeedforward armFeedforward;
  DutyCycleEncoder thruBore = new DutyCycleEncoder(2);
  double pidOutput;

  @Override
  public void robotInit() {

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_encoder = m_motor.getEncoder();

    m_motor.setInverted(false);

    // Constraints Coefficients
    maxVel = 180;
    maxAcc = 90;

    // PID coefficients
    kP = 0.004;
    kD = 0;
    kV = 0.0;

    setPoint = 30;
    // setPoint = 0;

    profiledPIDController = new ProfiledPIDController(kP, 0, kD,
        new TrapezoidProfile.Constraints(maxVel, maxAcc));

    profiledPIDController.setGoal(new State(setPoint, 0));

    armFeedforward = new ArmFeedforward(0, 0, kV);

    thruBore.setDistancePerRotation(-360);
    thruBore.setPositionOffset(0.659);

    currentPosition = thruBore.getDistance();

    profiledPIDController.reset(currentPosition);

    SmartDashboard.putNumber("setpoint", setPoint);

  }

  @Override
  public void robotPeriodic() {

    currentPosition = thruBore.getDistance();

    SmartDashboard.putNumber("current pos", currentPosition);
    SmartDashboard.putNumber("absolute pos", thruBore.getAbsolutePosition());

    pidOutput = profiledPIDController.calculate(currentPosition,
        setPoint);
    SmartDashboard.putNumber("pid tilt output", pidOutput);

  }

  @Override
  public void teleopPeriodic() {

    m_motor.set(pidOutput);

    // double pidVelocitySetpoint = profiledPIDController.getSetpoint().velocity;
    // SmartDashboard.putNumber("desiredVelocity", pidVelocitySetpoint);

    // double ffOutput = armFeedforward.calculate(pidVelocitySetpoint);
    // SmartDashboard.putNumber("ffOutput", ffOutput);

    // m_motor.set(MathUtil.clamp(MathUtil.applyDeadband(-new
    // Joystick(0).getRawAxis(1), 0.1), -0.15, 0.15));

    // m_motor.setVoltage(pidOutput + ffOutput);

  }
}