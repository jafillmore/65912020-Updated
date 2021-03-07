/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConst;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  private VictorSPX leftArm = new VictorSPX(ClimbConst.leftArmMotor);
  private VictorSPX rightArm = new VictorSPX(ClimbConst.rightArmMotor);

 

  public void turnOffBalanceMotors() {
    leftArm.set(ControlMode.PercentOutput, 0);
    rightArm.set(ControlMode.PercentOutput, 0);

  }

  public void turnOnBalanceMotors(double power) {
    leftArm.set(ControlMode.PercentOutput, power);
    rightArm.set(ControlMode.PercentOutput, power);

  }

  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
