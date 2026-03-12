//Package
package frc.robot.commands;

//Imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

//Create class
public class IntakePos extends Command {
    
    //Atrubutes
    private final IntakeSubsystem sub;
    private final double pos;

    //Constructor
    public IntakePos(IntakeSubsystem sub, double pos) {
        this.sub = sub;
        this.pos = pos;
    }

    @Override
    public void execute() {
        // Enviar la posición objetivo al subsistema en cada ciclo
        sub.setPosition(pos);
    }

    @Override
    public void end(boolean isFinished) {
        sub.setArmSpeed(0.025); // torque de sostenimiento contra la gravedad (solo NEOs del brazo)
    }

    @Override
    public boolean isFinished() {
        // Terminar cuando el mecanismo esté dentro de 0.1 rotaciones del objetivo
        return Math.abs(pos - sub.getPosition()) <= 0.1;
    }
}
