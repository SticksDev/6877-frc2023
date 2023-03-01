// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armLift extends SubsystemBase {
  private SparkMaxPIDController m_ArmPidController;
  private CANSparkMax m_armMotor;
  private final RelativeEncoder m_angleEncoder;

  /** Creates a new armLift. */
  public armLift() {
    m_armMotor = new CANSparkMax(Constants.kArmAngleChannel, MotorType.kBrushless);
    m_armMotor.setInverted(Constants.kArmAngleInverted);
    m_ArmPidController = m_armMotor.getPIDController();
    m_angleEncoder = m_armMotor.getEncoder();
    m_armMotor.setSmartCurrentLimit(30, 20, 0);

    m_ArmPidController.setP(Constants.kArmAngleP);
    m_ArmPidController.setI(Constants.kArmAngleI);
    m_ArmPidController.setD(Constants.kArmAngleD);
    m_ArmPidController.setIZone(Constants.kArmAngleIz);
    m_ArmPidController.setFF(Constants.kArmAngleFF);
    m_ArmPidController.setOutputRange(Constants.kArmAngleMinOutput, Constants.kArmAngleMaxOutput);

    m_armMotor.burnFlash();
  }

  public double getPosition() {
    return m_angleEncoder.getPosition();
  }

  public void move(double speed) {
    m_armMotor.set(speed);
    SmartDashboard.putNumber("Arm Angle Position", m_angleEncoder.getPosition());
  }

  public void resetEncoder() {
    m_angleEncoder.setPosition(0);
  }

  public void halt() {
    m_armMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
