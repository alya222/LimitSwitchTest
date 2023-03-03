// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Motor;

// import constants
import static frc.robot.Constants.*;

public class RestructuredSetMotorPosition extends CommandBase {
  /** Creates a new RestructuredSetMotorPosition. */

  private final Motor motor;

  public RestructuredSetMotorPosition(Motor motorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    motor = motorSubsystem;
    addRequirements(motor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.setMotorAutoSpeed(.3, 100, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motor.isEncoderAtSetPosition(motorRotations, 2);
  }
}
