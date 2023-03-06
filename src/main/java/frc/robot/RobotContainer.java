// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.MoveMotorToPosition;
import frc.robot.commands.MoveWristIn;
import frc.robot.commands.MoveWristOut;
import frc.robot.commands.SetMotorPosition;
import frc.robot.commands.PIDMoveMotorToPosition;
import frc.robot.commands.RunMotor;
import frc.robot.subsystems.Motor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj.XboxController.Button.*;


import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Drive Controller
  private XboxController xbox = new XboxController(kXboxPort);

  // The robot's subsystems and commands are defined here...

  private final Motor motor = new Motor();

  private final RunMotor runMotor = new RunMotor(motor);

  private final MoveMotorToPosition moveMotorToPosition = new MoveMotorToPosition(motor);

  private final SetMotorPosition setMotorPosition = new SetMotorPosition(motor);

  private final PIDMoveMotorToPosition pidMoveMotorToPosition = new PIDMoveMotorToPosition(motor, 10);

  private final MoveWristIn moveWristIn = new MoveWristIn(motor);

  private final MoveWristOut moveWristOut = new MoveWristOut(motor);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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

  
    // move motor a set number of rotations with one press
    /*new JoystickButton(xbox, kY.value)
    .onTrue(moveMotorToPosition);*/

    // motor runs forward while the button is held
    new JoystickButton(xbox, kA.value)
    .whileTrue(runMotor);

    // move to set position each time 
    /*new JoystickButton(xbox, kX.value)
    .onTrue(pidMoveMotorToPosition);*/

    // move to set position (100 encoder location) each time 
    new JoystickButton(xbox, kB.value)
    .onTrue(setMotorPosition);

    // move wrist to 130 encoder location
    new JoystickButton(xbox, kLeftBumper.value)
    .onTrue(moveWristOut);

    // move wrist to 0 encoder location
    new JoystickButton(xbox, kRightBumper.value)
    .onTrue(moveWristIn);




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
