// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.KrakenSwerveModule;
import frc.robot.subsystems.swerve.KrakenSwerveModule;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  CommandXboxController ps5Controller = new CommandXboxController(0); 

  private final KrakenSwerveModule swerveModule = new KrakenSwerveModule(1, 2, 0.0, 3);// dont change to SwerveModule or it will use CTRE swerve
  private SingleModuleSwerveSubsystem singleModuleSwerveSubsystem = new SingleModuleSwerveSubsystem(swerveModule);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ps5Controller.leftBumper().onTrue(new InstantCommand(() -> {
      singleModuleSwerveSubsystem.setState(0.1, Math.PI);
    }));

    ps5Controller.rightBumper().onTrue(new InstantCommand(() -> {
      singleModuleSwerveSubsystem.setState(-0.1, 0);
    }));

  

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }
}
