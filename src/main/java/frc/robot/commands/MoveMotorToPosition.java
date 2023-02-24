// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Motor;

// import constants
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class MoveMotorToPosition extends SequentialCommandGroup {

  // Drive Subsystem
  private final Motor motor;


  /** Creates a new MoveAndBalance. */
  public MoveMotorToPosition(Motor motorSubsystem) {

    motor = motorSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> motor.getTestMotorEncoder().setPosition(0), motor),
      new RunCommand(() -> motor.setSpeed(0.3))
      .until(() -> motor.isEncoderAtPosition(motorRotations)).until(motor::limitSwitchPressed),
      new RunCommand(() -> motor.setSpeed(0)).withTimeout(.1)

    );
  }
}
