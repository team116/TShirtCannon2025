// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;

  private final TalonFX frontLeftTalon = new TalonFX(1, "rio");
  private final TalonFX frontRightTalon = new TalonFX(3, "rio");
  private final TalonFX backLeftTalon = new TalonFX(2, "rio");
  private final TalonFX backRightTalon = new TalonFX(4, "rio");

  private final Follower frontLeftTalonFollower = new Follower(1, false);
  private final Follower frontRightTalonFollower = new Follower(3, false);

  private TalonFXConfiguration frontLeftConfig = new TalonFXConfiguration();
  private TalonFXConfiguration frontRightConfig = new TalonFXConfiguration();
  private TalonFXConfiguration backLeftConfig = new TalonFXConfiguration();
  private TalonFXConfiguration backRightConfig = new TalonFXConfiguration();

  private final Compressor compressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
  //private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    frontLeftConfig
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));

    frontRightConfig
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));    

    backLeftConfig
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));
        
    backRightConfig
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake));
  
    frontLeftTalon.getConfigurator().apply(frontLeftConfig);
    frontRightTalon.getConfigurator().apply(frontRightConfig);
    backLeftTalon.getConfigurator().apply(backLeftConfig);
    backRightTalon.getConfigurator().apply(backRightConfig);

    backLeftTalon.setControl(frontLeftTalonFollower);
    backRightTalon.setControl(frontRightTalonFollower);

    m_robotDrive = new DifferentialDrive(frontLeftTalon::set, frontRightTalon::set);
    m_leftStick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_leftStick.getRawAxis(1), -m_leftStick.getRawAxis(5));  // Raw Axis values for Logitech F310
  }
}
