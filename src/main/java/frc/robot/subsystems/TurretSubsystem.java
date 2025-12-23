package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableControls.TunableProfiledController;
import frc.robot.util.LoggedTunableNumber;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.vision.LimelightHelpers;



public class TurretSubsystem extends SubsystemBase {

  SparkFlex turretMotor = new SparkFlex(31, MotorType.kBrushless);

    private final LoggedTunableNumber kP =
   new LoggedTunableNumber("Turret/kP", 0.010);
   private final LoggedTunableNumber kI =
   new LoggedTunableNumber("Turret/kI", 0);
   private final LoggedTunableNumber kD =
   new LoggedTunableNumber("Turret/kD", 0);
   private final PIDController pid =
         new PIDController(kP.get(), kI.get(), kD.get());
    
public TurretSubsystem(){ 
   final SparkFlexConfig motorConfig;
  motorConfig = new SparkFlexConfig();
      motorConfig.closedLoop
            // Set PID values for velocity control in slot 1
            .p(0.1, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
            motorConfig.inverted(true);
            turretMotor.configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pid.setTolerance(1);
}

public void rotateRight(){
  turretMotor.set(0.2);
}

public void rotateLeft(){
  turretMotor.set(-0.2);
}

public void periodic(){
 
}



public void setTurn(){
   double target = LimelightHelpers.getTX("");
    double output = pid.calculate(target, 0); 
    
    SmartDashboard.putNumber("Target", target);
    SmartDashboard.putNumber("output", output);
  
    turretMotor.set(output);
  }

  public boolean atTarget() {
    return pid.atSetpoint();
  }


public Command Stop(){
  return runOnce(() -> {
    turretMotor.set(0);
  });
}




}
