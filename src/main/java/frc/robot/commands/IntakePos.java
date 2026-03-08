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
        //obtener las dos posibles posiciones, segun PositionVoltage: adentro, y afuera
        if (pos <= 0.1) sub.setPosition(pos);
    }

    @Override
    public void end(boolean isFinished) {
        sub.setSpeed(0.025);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pos - sub.getPosition()) <= 0.15;
    }
}
