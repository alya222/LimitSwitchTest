// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Motor extends SubsystemBase {

  private CANSparkMax testMotor = new CANSparkMax(testMotorPort, MotorType.kBrushless);
  private RelativeEncoder testMotorEncoder = testMotor.getEncoder();

  
  // add a limit switch on lift (to detect full extension)
  private SparkMaxLimitSwitch forwardLimit = testMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  /** Creates a new ExampleSubsystem. */
  public Motor() {
    testMotor.restoreFactoryDefaults();
    testMotor.setIdleMode(IdleMode.kBrake);
    testMotorEncoder.setPosition(0);
    testMotor.burnFlash();

    forwardLimit.enableLimitSwitch(false);
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setSpeed(double speed) {
    testMotor.set(speed);
  }

  public RelativeEncoder getTestMotorEncoder () {
    return testMotorEncoder;
  }

  public Boolean isEncoderAtInPosition (double position) {
    // only works if moving in a positive direction
      return testMotorEncoder.getPosition() <= position;
    
  }

  public Boolean isEncoderAtOutPosition (double position) {
    // only works if moving in a positive direction
      return testMotorEncoder.getPosition() >= position;
    
  }

  public Boolean isEncoderAtSetPosition (double position, double tolerance) {
    // only works if moving in a positive direction
    if (motorShouldMovePositive(positionTolerance)) {

      return testMotorEncoder.getPosition() - tolerance >= position;
    
    } else {

      return testMotorEncoder.getPosition() + tolerance <= position;
    }
    
  }

  public void setMotorAutoSpeed (double speed, double position, double tolerance) {

    if (motorShouldMovePositive(positionTolerance)) {
      
      testMotor.set(speed);

    } else if (motorShouldMoveNegative(positionTolerance)) {
      
      testMotor.set(-speed);

    } else if (isEncoderInRange (motorRotations, positionTolerance)) {

      testMotor.set(0);

    } else {}

  }

  /* let's have the set motor autospeed just give us a negative or positive 1 */

  public double motorAutoSpeedSign (double goalPosition) {
    return (goalPosition - testMotorEncoder.getPosition()) 
    / (Math.abs(goalPosition - testMotorEncoder.getPosition()));
  }

  /*
   * if the motor should move positive, move the motor positive until it is at the position
   * if the motor should move negative, move the motor negative until it is at the position
   */

  public Boolean motorShouldMovePositive (double tolerance) {
    return motorRotations - testMotorEncoder.getPosition() > 0 + tolerance;
  }

  public Boolean motorShouldMoveNegative (double tolerance) {
    return motorRotations - testMotorEncoder.getPosition() < 0 + tolerance;
  }

  public Boolean isEncoderInRange (double position, double tolerance) {
    return testMotorEncoder.getPosition() - tolerance >= position && testMotorEncoder.getPosition() + tolerance <= position;
    
  }

  // method checks whether higher lift limit switch is pressed
  public Boolean limitSwitchPressed() {
    return forwardLimit.isLimitSwitchEnabled();
  }

  public Double getEncoderPosition () {
    return testMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putNumber("Motor Encoder Counts", getEncoderPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
