package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.Volts;


public class ShooterSubsystem {
private SparkFlex anglemotor = new SparkFlex(14, MotorType.kBrushless);
private  SparkFlex motorFollower = new SparkFlex(21, MotorType.kBrushless);
 private TalonFX ShooterLeader = new TalonFX(8);
 private SparkClosedLoopController closedLoopController;
 private SparkFlexConfig motorConfig;
 private LimelightHelpers limelight;
 

 private final LoggedTunableNumber anglemotorkP =
   new LoggedTunableNumber("motor1/kP", 0.1);
   private final LoggedTunableNumber anglemotorkI =
   new LoggedTunableNumber("motor1/kI", 0.1);
   private final LoggedTunableNumber anglemotorkD =
   new LoggedTunableNumber("motor1/kD", 0.1);
   private final PIDController anglemotorpid =
         new PIDController(anglemotorkP.get(), anglemotorkI.get(),anglemotorkD.get());

         private final LoggedTunableNumber motorFollowerkP =
         new LoggedTunableNumber("anglemotor/kP", 0.1);
         private final LoggedTunableNumber motorFollowerkI =
         new LoggedTunableNumber("anglemotor/kI", 0.1);
         private final LoggedTunableNumber motorFollowerkD =
         new LoggedTunableNumber("anglemotor/kD", 0.1);
         private final PIDController motorFollowerpid =
               new PIDController(motorFollowerkP.get(), motorFollowerkI.get(), motorFollowerkD.get());

               private final LoggedTunableNumber ShooterLeaderkP =
               new LoggedTunableNumber("FollowerLeft/kP", 0.1);
               private final LoggedTunableNumber ShooterLeaderkI =
               new LoggedTunableNumber("FollowerLeft/kI", 0.1);
               private final LoggedTunableNumber ShooterLeaderkD =
               new LoggedTunableNumber("FollowerLeft/kD", 0.1);
               private final PIDController ShooterLeaderpid =
                     new PIDController(ShooterLeaderkP.get(), ShooterLeaderkI.get(), ShooterLeaderkD.get());

      
      
         

public ShooterSubsystem() {
 TalonFXConfiguration configs = new TalonFXConfiguration();
configs.Slot0.kP = 2.4; 
configs.Slot0.kI = 0; 
configs.Slot0.kD = 0.1; 
configs.Voltage.withPeakForwardVoltage(Volts.of(8))
  .withPeakReverseVoltage(Volts.of(-8));

  final SparkFlexConfig motorConfig;
  motorConfig = new SparkFlexConfig();
closedLoopController = anglemotor.getClosedLoopController();
      motorConfig.closedLoop
            // Set PID values for velocity control in slot 1
            .p(0.05, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot1);
            motorConfig.inverted(true);
            anglemotor.configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            anglemotorpid.setTolerance(1);
         
             
             
          
            closedLoopController = motorFollower.getClosedLoopController();
                  motorConfig.closedLoop
                        // Set PID values for velocity control in slot 1
                        .p(0.05, ClosedLoopSlot.kSlot1)
                        .i(0, ClosedLoopSlot.kSlot1)
                        .d(0, ClosedLoopSlot.kSlot1)
                        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                        .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot1);
                        motorConfig.inverted(true);
                        motorFollower.configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                        motorFollowerpid.setTolerance(1);
}


  public void Shot(){
double target = LimelightHelpers.getTY("");

//SmartDashboard.putNumber("Target", target);
// SmartDashboard.putNumber("PID Turret: ", output);

//motorFollower.set(output);
//ShooterLeader.set(output);
 }
  
  
}






