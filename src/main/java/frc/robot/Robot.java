/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
public class Robot extends TimedRobot {

  private static final int kMotorPort = 0;
  private static final int kJoystickPort = 0;

  private CANSparkMax neo;
  private CANEncoder neo_encoder;
  private CANPIDController neo_pidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private Joystick m_joystick;


  @Override
  public void robotInit() {

    neo = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    neo.restoreFactoryDefaults();

    neo_encoder = neo.getEncoder();

    m_joystick = new Joystick(kJoystickPort);

    // PID Constants
    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Constants
    maxVel = 0;
    minVel = 0;
    maxAcc = 0;
    allowedErr = 0;

    neo_pidController.setP(kP);
    neo_pidController.setI(kI);
    neo_pidController.setD(kD);
    neo_pidController.setIZone(kIz);
    neo_pidController.setFF(kFF);
    neo_pidController.setOutputRange(kMinOutput, kMaxOutput);


    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {

    neo.set(m_joystick.getY());


    //GetPosition() returns the position in units of revolutions
    SmartDashboard.putNumber("Encoder Position", neo_encoder.getPosition());

    //GetVelocity() returns the velocity in RPM
    SmartDashboard.putNumber("Encoder Velocity", neo_encoder.getVelocity());

  }

  @Override
  public void teleopPeriodic() {

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double setPoint = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { neo_pidController.setP(p); kP = p; }
    if((i != kI)) { neo_pidController.setI(i); kI = i; }
    if((d != kD)) { neo_pidController.setD(d); kD = d; }
    if((iz != kIz)) { neo_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { neo_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      neo_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    neo_pidController.setReference(setPoint, ControlType.kPosition);

    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", neo_encoder.getPosition());
    SmartDashboard.putNumber("Output", neo.getAppliedOutput());

  }
}
