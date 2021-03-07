/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConst;


// Intake System
public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intakesubsystem.
   */

   //Intake Motors 
  private CANSparkMax liftMotor = new CANSparkMax(IntakeConst.liftMotor, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConst.intakeMotor, MotorType.kBrushless);

  

  //Intake Settings
  public void turnOnIntake() {
    intakeMotor.set(IntakeConst.intakeSpeed);
    liftMotor.set(IntakeConst.liftSpeed);
  }

  public void turnOffIntake() {
    intakeMotor.set(0);
    liftMotor.set(0);
  }


  public void reverseIntakeLift()  {
    liftMotor.set(-IntakeConst.liftSpeed);
  }

  public void turnOffReverseIntakeLift()  {
    liftMotor.set(0);
  }

  public IntakeSubsystem() {

    liftMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
