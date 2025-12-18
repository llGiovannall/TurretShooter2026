package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.Volts;


public class ShooterSubsystem {
private SparkFlex FollowerRight = new SparkFlex(14, MotorType.kBrushless);
private  SparkFlex FollowerLeft = new SparkFlex(21, MotorType.kBrushless);
 private TalonFX ShooterLeader = new TalonFX(8);
 private SparkClosedLoopController closedLoopController;
 private SparkFlexConfig motorConfig;
 private LimelightHelpers limelight;

 private final LoggedTunableNumber FollowerRightkP =
   new LoggedTunableNumber("FollowerRight/kP", 0.1);
   private final LoggedTunableNumber FollowerRightkI =
   new LoggedTunableNumber("FollowerRight/kI", 0.1);
   private final LoggedTunableNumber FollowerRightkD =
   new LoggedTunableNumber("FollowerRight/kD", 0.1);
   private final PIDController FollowerRightpid =
         new PIDController(FollowerRightkP.get(), FollowerRightkI.get(), FollowerRightkD.get());

         private final LoggedTunableNumber FollowerLeftkP =
         new LoggedTunableNumber("FollowerLeft/kP", 0.1);
         private final LoggedTunableNumber FollowerLeftkI =
         new LoggedTunableNumber("FollowerLeft/kI", 0.1);
         private final LoggedTunableNumber FollowerLeftkD =
         new LoggedTunableNumber("FollowerLeft/kD", 0.1);
         private final PIDController FollowerLeftpid =
               new PIDController(FollowerLeftkP.get(), FollowerLeftkI.get(), FollowerLeftkD.get());

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


  closedLoopController = FollowerLeft.getClosedLoopController();
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        closedLoopController = FollowerRight.getClosedLoopController();
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
            limelight = new LimelightHelpers();}

public void Shot(){
   // double distance =limelight.calculateDistance
//limelight.updateTracking();
}




}
