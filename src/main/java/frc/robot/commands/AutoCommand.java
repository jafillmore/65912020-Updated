/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.AutoConst;
import frc.robot.Constants.PIDConst;
import frc.robot.Constants.ShooterConst;
import frc.robot.subsystems.ArcadeDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommand.
   */
 
  public AutoCommand(ArcadeDriveSubsystem arcadeDriveSubsystem, ShooterSubsystem shooterSubsystem) {

    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    
    super(
      /*new StartEndCommand(
        // Drive Forward
        () -> arcadeDriveSubsystem.arcadeDrive(AutoConst.AutoDriveSpeed, 0),
        // Stop Driving
        () -> arcadeDriveSubsystem.arcadeDrive(0, 0), arcadeDriveSubsystem)
      // Reset Encoder
      .beforeStarting(arcadeDriveSubsystem :: resetEncoders, arcadeDriveSubsystem)
      // End The Command
      .withInterrupt(() -> arcadeDriveSubsystem.getAverageEncoderDistance() >= AutoConst.AutoDriveDistanceInches)

      //new RunCommand(() -> shooterSubsystem.targetAndShoot(), shooterSubsystem)
    );  */
      new SequentialCommandGroup());
      new RunCommand(() -> shooterSubsystem.targetAndShoot())
      .withTimeout(10).andThen(new RunCommand(() -> arcadeDriveSubsystem.arcadeDrive(.25, 0)));
  }
}
