// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motor;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunMotor extends CommandBase {
  private final Motor motor;

  /**
   * Creates a new ExampleCommand.
   *
   * @param motorSubsystem The subsystem used by this command.
   */
  public RunMotor(Motor motorSubsystem) {
    motor = motorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setSpeed(.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
