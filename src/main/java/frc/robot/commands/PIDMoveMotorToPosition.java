// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PIDMoveMotorToPosition extends CommandBase {
  
  private final Motor motorSubsystem;
  private PIDController motorPIDController;

  private double setPoint;


  /**
   * Creates a new ExampleCommand.
   *
   * @param motorSubsystem The subsystem used by this command.
   */
  public PIDMoveMotorToPosition(Motor motorSubsystem, double setPoint) {
    this.motorSubsystem = motorSubsystem;
    motorPIDController = new PIDController(.0004, 0., 0.0);

    motorPIDController.setTolerance(0.0035);
    
    this.setPoint = setPoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double feedforward = -0.05;

    double speed = motorPIDController.calculate(motorSubsystem.getEncoderPosition(), setPoint);

    /* The code is a shortcut for the following:
     * if (speed > 0) { speed = speed + feedforward }
     * else { speed = speed - feedforward } 
     * 
     * if (speed > 1) { speed = 1.0 }
     * else { speed = speed } 
     * 
     * if (speed < -1) { speed = -1.0 }
     * else { speed = speed } 
     */

    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    speed = (speed > 1 ) ? 1.0 : speed;
    speed = (speed < -1 ) ? -1 : speed; 

    motorSubsystem.setSpeed(speed);

    SmartDashboard.putNumber("motor output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorSubsystem.isEncoderInRange(setPoint, 5);  
  }

  public void setPoint(double setPoint) {
    this.setPoint = setPoint;
  }

}
