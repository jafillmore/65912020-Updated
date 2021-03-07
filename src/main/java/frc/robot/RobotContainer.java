/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.Constants.JoystickConst;
import frc.robot.Constants.PIDConst;
import frc.robot.Constants.ShooterConst;
import frc.robot.commands.AutoCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArcadeDriveSubsystem arcadeDriveSubsystem = new ArcadeDriveSubsystem();
  private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final Command autoCommand = new AutoCommand(arcadeDriveSubsystem, shooterSubsystem);
  
  // DO NOT REMOVE - Required to get the cameras started....
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  // DO NOT REMOVE - Required to get the cameras started....

  Joystick leftJoystick = new Joystick(JoystickConst.leftJoystickPort);
  Joystick rightJoystick = new Joystick(JoystickConst.rightJoystickPort);
  Joystick joeStick = new Joystick(JoystickConst.joeStickPort);

   /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   configureButtonBindings();
     
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //ArcadeDriveSubsystem Joysticks 
    
    //  Drive the Robot        
    arcadeDriveSubsystem.setDefaultCommand(
      new RunCommand(() -> arcadeDriveSubsystem
          .arcadeDrive(leftJoystick.getY(), 
                       rightJoystick.getZ()), arcadeDriveSubsystem));

    /////////////////////   Intake Stuff   //////////////////////////
    //  Extend / Retract Intake Arms                 
    new JoystickButton(leftJoystick, JoystickConst.toggleIntake)
      .whenHeld(new InstantCommand(pneumaticSubsystem:: deployIntake))
      .whenReleased(new InstantCommand(pneumaticSubsystem::stowIntake));

    // Turn on Intake motors
    new JoystickButton(rightJoystick, JoystickConst.intakeTrigger)
      .whileHeld(new InstantCommand(intakeSubsystem:: turnOnIntake))
      .whenReleased(new InstantCommand(intakeSubsystem:: turnOffIntake));

    // Reverse Intake Lift Motor
    new JoystickButton(rightJoystick, JoystickConst.intakeReverse)
      .whenHeld(new InstantCommand(intakeSubsystem::reverseIntakeLift))
      .whenReleased(new InstantCommand(intakeSubsystem::turnOffIntake));
    /////////////////////  End of Intake Stuff   //////////////////////////////
    
    ////////////////////   Climbing Stuff  ////////////////////////////////////
    // Extend Climb Arms to prepare for climbing
    new JoystickButton(joeStick, JoystickConst.extendClimbArm)
      .whenPressed(new InstantCommand(pneumaticSubsystem::extendClimbArms));

    // Retract Climb Arms (aka Climb!)
    new JoystickButton(joeStick, JoystickConst.retractClimbArm)
      .whenPressed(new InstantCommand(pneumaticSubsystem::retractClimbArms));

    // Balance using the joystick
    climberSubsystem.setDefaultCommand(
      new RunCommand(() -> climberSubsystem.turnOnBalanceMotors(joeStick.getRawAxis(0)), climberSubsystem));
    
    // Deploy (flip up) Climb arms
    new JoystickButton(joeStick, JoystickConst.deployClimbArm)
      .whenPressed(new InstantCommand(pneumaticSubsystem::deployClimbArms));

    // Stow (fold down) Climb arms
    new JoystickButton(joeStick, JoystickConst.stowClimbArm)
      .whenPressed(new InstantCommand(pneumaticSubsystem::stowClimbArms));
    ////////////////////////  End of Climbing Stuff    //////////////////////////////

    ////////////////////////    Shooting Stuff   ////////////////////////////////
    /*
    // Automatic Targeting
    new JoystickButton(joeStick, JoystickConst.autoTarget)
    .whenPressed(new RunCommand(() -> shooterSubsystem.target()));  
    */

    // Low Power Fire
    new JoystickButton(joeStick, JoystickConst.slowFire)
    .whileHeld(new RunCommand(() -> shooterSubsystem.shooterOn(PIDConst.SlowStartingSpeed)))
      //.whileHeld(new RunCommand(() -> shooterSubsystem.shootOn()))
      //-> intakeSubsystem.liftSpeed(IntakeConst.liftShootSpeed)));
      .whenReleased (new InstantCommand(() -> shooterSubsystem.shootMotorOff()));

    // Full Power Fire
    new JoystickButton(joeStick, JoystickConst.fastFire)
      .whileHeld(new RunCommand(() -> shooterSubsystem.shooterOn(PIDConst.FastStartingSpeed)))
      .whenReleased( new InstantCommand(() -> shooterSubsystem.shootMotorOff()));

    // Increase Low Power Shot speed
    new JoystickButton(joeStick, JoystickConst.increaseSpeed)
      .whenPressed(new InstantCommand(() -> shooterSubsystem.adjShooterSpeedUp()));

    // Decrease Low Power Shot Speed
    new JoystickButton(joeStick, JoystickConst.decreaseSpeed)
      .whenPressed(new InstantCommand(() -> shooterSubsystem.adjShooterSpeedDown()));

    // Manual Prime
    new JoystickButton(joeStick, JoystickConst.firePrimeMotor)
      .whileHeld(new RunCommand(() -> shooterSubsystem.primeMotorOn()))
      .whenReleased(new InstantCommand(() -> shooterSubsystem.shootMotorOff()));
    
    new JoystickButton(joeStick, 15)
      .whileHeld(new RunCommand(() -> shooterSubsystem.reversePrimeMotor()))
      .whenReleased(new InstantCommand(() -> shooterSubsystem.shootMotorOff()));

    new JoystickButton(joeStick,21)
      .whenPressed(new RunCommand(() -> shooterSubsystem.target()));
      
    new JoystickButton(joeStick,20)
      .whenPressed(new RunCommand(() -> shooterSubsystem.target()));
    

    // Default Command for rotating the shooter   
    shooterSubsystem.setDefaultCommand(
      new RunCommand(() -> shooterSubsystem .rotate(joeStick.getRawAxis(2)), shooterSubsystem));

    // Show the shooter motor speed on the Driver Station
    Shuffleboard.getTab("Actual Shooter RPM").add("rpm", shooterSubsystem.encoder.getVelocity());
    SmartDashboard.putNumber("Ideal Shooter RPM", shooterSubsystem.shooterSpeed);
    
    ///////////////////////////   End of Shooter Stuff   //////////////////////////////////


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
