// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

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
    testMotor.burnFlash();

    forwardLimit.enableLimitSwitch(false);
    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
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

  public Boolean isEncoderAtPosition (double position) {
      return testMotorEncoder.getPosition() >= position;
  }

  // method checks whether higher lift limit switch is pressed
  public boolean limitSwitchPressed() {
    return forwardLimit.isLimitSwitchEnabled();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
