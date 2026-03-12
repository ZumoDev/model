package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    private final ShooterSubsystem sub;
    private final double speed;
    private boolean isCalculationEnabled;
    private final IntakeSubsystem intakeSub;
    private final Supplier<LimelightHelpers.LimelightTarget_Fiducial> closestTagSupplier;
    
    public StartShooter(ShooterSubsystem sub, IntakeSubsystem intakeSub, double speed, boolean isCalculationEnabled, Supplier<LimelightHelpers.LimelightTarget_Fiducial> closestTagSupplier) {
        this.sub = sub;
        this.intakeSub = intakeSub;
        this.speed = speed;
        this.isCalculationEnabled = isCalculationEnabled;
        this.closestTagSupplier = closestTagSupplier;
        addRequirements(sub, intakeSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var tag = closestTagSupplier.get();
        String limelightName = Constants.Limelights.shooterLimelightName;

        if (sub.isVelocityWithinTolerance()) {
            intakeSub.setIndexerSpeed(0.75);

            if (isCalculationEnabled || tag != null) {
                Pose3d targetLocation = tag.getTargetPose_CameraSpace();
                double tz = targetLocation.getZ();

                if (LimelightHelpers.getTV(limelightName)) sub.setRPM(
                    sub.calculateRPMWithITM(limelightName, tz), 
                    sub.rsMotor
                );
            }

            else sub.set(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
