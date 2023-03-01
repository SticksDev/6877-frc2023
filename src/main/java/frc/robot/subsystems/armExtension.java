// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class armExtension extends SubsystemBase {

  private final CANSparkMax m_extensionMotor;
  private final SparkMaxPIDController m_extensionPID;
  private final RelativeEncoder m_extensionEncoder;

  /** Creates a new armExtension. */
  public armExtension() {
    m_extensionMotor = new CANSparkMax(Constants.kArmExtensionChannel, MotorType.kBrushless);
    m_extensionMotor.setInverted(Constants.kArmExtensionInverted);
    m_extensionPID = m_extensionMotor.getPIDController();
    m_extensionEncoder = m_extensionMotor.getEncoder();
    m_extensionMotor.setSmartCurrentLimit(30, 20, 0);

    m_extensionPID.setP(Constants.kArmExtensionP);
    m_extensionPID.setI(Constants.kArmExtensionI);
    m_extensionPID.setD(Constants.kArmExtensionD);
    m_extensionPID.setIZone(Constants.kArmExtensionIz);
    m_extensionPID.setFF(Constants.kArmExtensionFF);
    m_extensionPID.setOutputRange(Constants.kArmExtensionMinOutput, Constants.kArmExtensionMaxOutput);

    m_extensionMotor.burnFlash();
  }

  public void setLength(double length) {
    // do some math to calibrate angle into a position
    double position = length;
    m_extensionPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void extendArm() {
    System.out.println("Calling extend");
    m_extensionPID.setReference(580, CANSparkMax.ControlType.kSmartMotion);
  }

  public void retractArm() {
    System.out.println("Calling Retract");
    m_extensionPID.setReference(0, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getPosition() {
    return m_extensionEncoder.getPosition();
  }

  public void move(double speed) {
    m_extensionMotor.set(speed);
    SmartDashboard.putNumber("Extension Position", m_extensionEncoder.getPosition());
  }

  public void resetEncoder() {
    m_extensionEncoder.setPosition(0);
  }

  public void halt() {
    m_extensionMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
