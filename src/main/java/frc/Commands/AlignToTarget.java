package frc.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class AlignToTarget extends Command {
private final Turret  turret;

public AlignToTarget(Turret turret){
    this.turret = turret;
}

public void execute(){
turret.setTurn();
}


}
