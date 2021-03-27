/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

public class ArcadeDriveSubsystem extends SubsystemBase {
  public static final String robotDrive = null;
/**
   * Creates a new ArcadeDriveSubsystem.
   */
  // Motor Types
  public CANSparkMax frontLeft = new CANSparkMax(DriveConst.frontLeftMotor, MotorType.kBrushless);
  public CANSparkMax midLeft = new CANSparkMax(DriveConst.midLeftMotor, MotorType.kBrushless);
  public CANSparkMax frontRight = new CANSparkMax(DriveConst.frontRightMotor, MotorType.kBrushless);
  public CANSparkMax midRight = new CANSparkMax(DriveConst.midRightMotor, MotorType.kBrushless);

  public CANEncoder frontLeftEncoder = new CANEncoder(frontLeft);
  public CANEncoder midLeftEncoder = new CANEncoder(midLeft);
  public CANEncoder frontRightEncoder = new CANEncoder(frontRight);
  public CANEncoder midRightEncoder = new CANEncoder(midRight);

  public double averageEncoderDistance;
  
 

  // Speed Controller Group's 
  public SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeft, midLeft);
  public SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRight, midRight);

  //DifferentialGroup 
  public DifferentialDrive robotdrive = new DifferentialDrive(leftMotors, rightMotors);

  //Joystick
  //public Joystick mainJoystick = new Joystick(0);

  public void arcadeDrive(double fwd, double rot) {
  robotdrive.arcadeDrive(-fwd, rot*Math.abs(rot));  /**DriveConst.turnSpeed*/
  }

  public void resetEncoders(){
    frontLeftEncoder.setPosition(0);
    midLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    midRightEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance(){
    double posOfFL = frontLeftEncoder.getPosition();
    double posOfML = midLeftEncoder.getPosition();
    double posOfFR = frontRightEncoder.getPosition();
    double posOfMR = midRightEncoder.getPosition();

    averageEncoderDistance = (posOfFL + posOfML + posOfFR + posOfMR) / 4;
    return averageEncoderDistance;
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    robotdrive.setDeadband(0.02);
  }

  
}
