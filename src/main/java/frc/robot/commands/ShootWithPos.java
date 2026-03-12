package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AprilTags.*;

public class ShootWithPos extends Command {
    private final ShooterSubsystem sub;
    private final Supplier<LimelightHelpers.LimelightTarget_Fiducial> closestTagSupplier;

    public ShootWithPos(ShooterSubsystem sub, Supplier<LimelightHelpers.LimelightTarget_Fiducial> closestTagSupplier) {
        this.sub = sub;
        this.closestTagSupplier = closestTagSupplier;
        addRequirements(sub);
    }

    @Override
    public void execute() {
        var limelight = Constants.Limelights.shooterLimelightName;
        LimelightHelpers.LimelightTarget_Fiducial tag = null;
        Pose3d robotPos = LimelightHelpers.getBotPose3d(limelight);

        if (LimelightHelpers.getTV(limelight)) tag = closestTagSupplier.get();
        else return;

        if (tag != null) {
            Pose3d targetPos = closestTagSupplier.get().getTargetPose_CameraSpace();
            double error = Math.abs(targetPos.getZ() - robotPos.getZ());

            double targetRPM = sub.calculateRPMWithITM(limelight, error);
            sub.setRPM(targetRPM, sub.rsMotor);
        }
    }
}
