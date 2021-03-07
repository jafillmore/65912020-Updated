/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConst;


public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ColorsensorSubsystem.
   */
  //ControlPanel Motor
  public VictorSPX control = new VictorSPX(ControlPanelConst.spinningMotor);

  // Color Sensor's
  public ColorSensorV3 colorSensor = new ColorSensorV3(ControlPanelConst.colorSensorPort);
  public ColorMatch colorMatcher = new ColorMatch();
  public Color detectedColor = colorSensor.getColor();

 /**
    * Run the color match algorithm on our detected color
    */
    public ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

  //Color Points
  public ControlPanelSubsystem() {
    colorMatcher.addColorMatch(ControlPanelConst.blueTarget);
    colorMatcher.addColorMatch(ControlPanelConst.greenTarget);
    colorMatcher.addColorMatch(ControlPanelConst.redTarget);
    colorMatcher.addColorMatch(ControlPanelConst.yellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
