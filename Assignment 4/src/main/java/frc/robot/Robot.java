// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final PWMSparkMax m_shooter = new PWMSparkMax(2);
  private final PWMSparkMax m_intake = new PWMSparkMax(3);
  private final Encoder m_encoder = new Encoder(0, 1);
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private int count = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0); //Setting distance per pulse to circumference of wheel
  
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_encoder.reset();
    m_gyro.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (count == 0) {
      if (m_encoder.getDistance() < 84) { //Assuming the ball is 84 inches away (7 feet)
        m_robotDrive.arcadeDrive(0.5, 0.0);
        m_intake.set(0.5);
      }
      else {
        m_intake.stopMotor();
        count ++;
      }
    }

    else if (count == 1) {
      if (m_gyro.getAngle() < 90.0) {
        m_robotDrive.arcadeDrive(0.0, 0.5);
      }
      else {
        count ++;
        m_encoder.reset();
      }
    }

    else if (count == 2) {
      if (m_encoder.getDistance() < 210) {
        m_robotDrive.arcadeDrive(0.5, 0.0);
        m_intake.set(0.5);
      }
      else {
        m_intake.stopMotor();
        m_gyro.reset();
        count ++;
      }
    }

    else if (count == 3) {
      if (m_gyro.getAngle() < -45) {
        m_robotDrive.arcadeDrive(0.0, -0.5);
      }
      else {
        count ++;
        m_robotDrive.stopMotor();
      }
    }

    else if (count==4) {
      if (m_timer.get() < 3.0) {
        m_shooter.set(0.5);
      }
      else {
        m_shooter.stopMotor();
        count ++ ;
      }
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    if (m_stick.getRawButton(0)) {
      m_intake.stopMotor();
    }
    
    if (m_stick.getRawButton(1)) {
      m_intake.set(0.5);
    }

    if (m_stick.getRawButton(2)) {
      m_intake.set(-0.5);
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
