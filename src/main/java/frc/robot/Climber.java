package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  DigitalInput climberLowSensor; // Sensor that detects when the climber gets to low position

  private final TalonFX RightClimberMotor;

  public Climber() {
    RightClimberMotor = new TalonFX(14);
    RightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberLowSensor = new DigitalInput(3);
  }

  // moves the climber down (motor runs forward)
  public Command Run() {
    return this.run(() -> RightClimberMotor.set(1));
  }
  
  // moves the climber up (motor runs backward)
  public Command Reverse() {
    return this.run(() -> RightClimberMotor.set(-1));
  }
  
  // stops the motor
  public Command Stop() {
    return this.runOnce(() -> RightClimberMotor.set(0));
  }

  @Override
  public void periodic() {
    // Get current position
    double currentPosition = RightClimberMotor.getPosition().getValueAsDouble();
    
    // log climber sensor state
    SmartDashboard.putBoolean("climber sensor", climberLowSensor.get());
    SmartDashboard.putNumber("climber position", currentPosition);

    // reset climber motor integrated encoder position to 0 if climber reaches low point
    if (!climberLowSensor.get()) {
      RightClimberMotor.setPosition(0);
    }
    
    // Safety: stop motor if it goes past upper limit
    if (currentPosition < -273) {
      RightClimberMotor.set(0);
    }
  }
}