/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PnemuaticConst;

public class PneumaticSubsystem extends SubsystemBase {
  
/**
   * Creates a new PneumaticSubsystem.
   */
  Compressor c = new Compressor(0);


  private static DoubleSolenoid deployArmsDouble = new DoubleSolenoid(PnemuaticConst.deployA, PnemuaticConst.deployB);
  private static DoubleSolenoid extendArmsDouble = new DoubleSolenoid(PnemuaticConst.extandA, PnemuaticConst.extandB);
  private static DoubleSolenoid deployIntakeDouble = new DoubleSolenoid(PnemuaticConst.intakeA, PnemuaticConst.intakeB);

  private static AnalogInput pressureSensor = new AnalogInput(3);
  
  private double returnedVoltage = 0.0;
  private double calculatedPressure =0.0;
  
  
  public PneumaticSubsystem() {
    pressureSensor.setAverageBits(4);
    pressureSensor.setOversampleBits(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    printPressure();

  }


  public void printPressure(){

    returnedVoltage = pressureSensor.getAverageVoltage();
    calculatedPressure = 250*(returnedVoltage/PnemuaticConst.normalizedVoltage)-25;

    SmartDashboard.putNumber("Average Voltage", pressureSensor.getAverageVoltage());
    SmartDashboard.putNumber("Get Voltage", pressureSensor.getVoltage());
    SmartDashboard.putNumber("Returned Voltage", returnedVoltage);
    SmartDashboard.putNumber("Pressure", calculatedPressure);
    
  }


//***************************************************** */
//deploy the climb
  public void deployClimbArms() {
    deployArmsDouble.set(Value.kOff);  
    deployArmsDouble.set(Value.kForward);  
  } 

//stow the Climb Arms
  public void stowClimbArms(){
    deployArmsDouble.set(Value.kOff);
    deployArmsDouble.set(Value.kReverse);
  }


 //********************************************* */ 
//extand climb arms
  public void extendClimbArms(){
    extendArmsDouble.set(Value.kOff);
    extendArmsDouble.set(Value.kForward);    
  }

//Retract climb arms (aka:  Climb!)
  public void retractClimbArms(){
    extendArmsDouble.set(Value.kOff);
    extendArmsDouble.set(Value.kReverse);  
}

// deploy intake

  public final void deployIntake(){
    deployIntakeDouble.set(Value.kOff);
    deployIntakeDouble.set(Value.kForward);
    //deployIntakeDouble.set(Value.kOff);
    System.out.println("Deploying Intake");
  }

  public void stowIntake(){
    deployIntakeDouble.set(Value.kOff);
    deployIntakeDouble.set(Value.kReverse);
    //deployIntakeDouble.set(Value.kOff);
    System.out.println("Stowing Intake");
  }




}
