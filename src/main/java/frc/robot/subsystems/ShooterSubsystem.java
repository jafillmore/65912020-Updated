/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TargetPipeline;
import frc.robot.Constants.PIDConst;
import frc.robot.Constants.ShooterConst;
import frc.robot.Constants.VisConst;



public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final TargetPipeline targetPipeline = new TargetPipeline();
   //Create Shooter Motor
  private CANSparkMax shooterMotor = new CANSparkMax(ShooterConst.Shooter, MotorType.kBrushless);
  public CANSparkMax targetMotor = new CANSparkMax(ShooterConst.Targeting, MotorType.kBrushless);
  private CANSparkMax primeMotor = new CANSparkMax(ShooterConst.primeMotor, MotorType.kBrushless);
  public CANEncoder encoder = new CANEncoder(shooterMotor);
  public CANPIDController PID = new CANPIDController(shooterMotor);
  public CANEncoder targetingencoder = new CANEncoder(targetMotor);

  public double shooterSpeed = PIDConst.SlowStartingSpeed;
  public double fastShooterSpeed = PIDConst.FastStartingSpeed;

  public DigitalInput limitSwitch = new DigitalInput(ShooterConst.LimitSwitchPort);
  private boolean isBallPrimed = false;
  private boolean onTarget = false;
  
  public ShooterSubsystem() {
    PID.setP(PIDConst.P);
    PID.setI(PIDConst.I);
    PID.setD(PIDConst.D);
    PID.setIZone(PIDConst.Iz);
    PID.setFF(PIDConst.FF);
    PID.setOutputRange(PIDConst.MinOutput, PIDConst.MaxOutput);

    if (!shooterMotor.getInverted()){
      shooterMotor.setInverted(true);
      
    }

    targetMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printLimitSwitchStatus();

  }
  
  public void printLimitSwitchStatus() {
    SmartDashboard.putBoolean("Primer Limit SW Status", limitSwitch.get());
  }
  
  public void primeBall(){
    primeMotor.setInverted(false);
    
    if(!limitSwitch.get()){
      primeMotor.set(0);
      isBallPrimed = true;
      return;
    } else {
        primeMotor.set(ShooterConst.primeMotorPrimeSpeed);
        if(!limitSwitch.get()){
          primeMotor.set(0);
          isBallPrimed = true;
          return;
        } else {
          isBallPrimed = false;
        }
    }
  }

  public void primeMotorOn (){
    primeMotor.set(ShooterConst.primeMotorShootSpeed);
  }
  

  ////////////////////////////////////     New Shooter Command (if it doesn't work it is Jade's fault...)  //////////////
  //If this method does not work, uncomment the methods above and change the method that the button press calls in RobotContainer
  public void shooterOn (double speedOfShooter){
    shooterMotor.setInverted(false);
    
    primeMotor.setInverted(false);

    PID.setReference(speedOfShooter, ControlType.kVelocity);

    SmartDashboard.putNumber("Actual Motor RPM", (encoder.getVelocity()));
    SmartDashboard.putNumber("Target Motor RPM", (speedOfShooter/3));
    
    /*if(!isBallPrimed){
      primeBall();
    } else {
*/
      if(encoder.getVelocity() >= (speedOfShooter/3 -PIDConst.AllowableSpeedError)){
        primeMotor.set(ShooterConst.primeMotorShootSpeed);
      } else if(encoder.getVelocity() <= speedOfShooter/3-PIDConst.AllowableSpeedError) {
        primeMotor.set(0);
      }
    //}
  }

  ////////////  Turn off Shooter Motor and Priming Motor ////////////////
  public void shootMotorOff(){
    shooterMotor.set(0);
    primeMotor.set(0);
  }

  public void reversePrimeMotor(){
    primeMotor.set(-ShooterConst.primeMotorShootSpeed);
  }
  

  ////////////   Manual Adjust Shooter Speed Up for Low Power Shots   //////////////////
  public void adjShooterSpeedUp(){
    shooterSpeed += 500;
    if (shooterSpeed >= PIDConst.FastStartingSpeed){shooterSpeed=PIDConst.FastStartingSpeed;}
    SmartDashboard.putNumber("Target Motor RPM", (shooterSpeed/3));
  } 
  ////////////   Manual Adjust Shooter Speed Down for Low Power Shots   //////////////////  
  public void adjShooterSpeedDown(){
    shooterSpeed -= 500;
    if (shooterSpeed <= PIDConst.SlowStartingSpeed){shooterSpeed=PIDConst.SlowStartingSpeed;}
    SmartDashboard.putNumber("Target Motor RPM", (shooterSpeed/3));
  } 

  ////////////////////   Rotate the Shooter   /////////////////////////////////  
  public void rotate(double chubby) {
    targetMotor.set(0.2*chubby);
  }

  ///////////////////   Auto Target the Shooter   //////////////////////////////
  
  public void target(){
    if (targetPipeline.filterContoursOutput().isEmpty()) {
      onTarget = false;
      return;
    }
  
    if (Math.abs(visionSubsystem.targetError) > VisConst.allowableTargetError) {
      rotate(visionSubsystem.targetError/(VisConst.TargetCameraFrameWidth/2));
      onTarget = false;
      return;
    }

    if (Math.abs(visionSubsystem.targetError) < VisConst.allowableTargetError) {
      rotate(0.0);
      onTarget = true;
      return;      
    }

    SmartDashboard.putBoolean("On Target", onTarget);
    
  }
  
  public void targetAndShoot() {
    target();
    if (onTarget) {
      shooterOn(PIDConst.FastStartingSpeed);
    }

  }
  
}