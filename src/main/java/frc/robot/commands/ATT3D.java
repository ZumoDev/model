package frc.robot.commands;


import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants.*;

public class ATT3D extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController xPID;
    private final PIDController zPID;
    private final PIDController turnPID;
    private final double maxAngularRate;
    private final double maxSpeed;
    private final String limelightName;
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public ATT3D(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest, PIDController xPID, PIDController zPID, PIDController turnPID, double maxAngularRate, double maxSpeed, String limelightName) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.xPID = xPID;
        this.zPID = zPID;
        this.turnPID = turnPID;
        this.maxAngularRate = maxAngularRate;
        this.maxSpeed = maxSpeed;
        this.limelightName = limelightName;

        addRequirements(drivetrain); 
    }

    @Override
    public void initialize() {
        zPID.setIntegratorRange(-0.2, 0.2);
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;

        //AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        //Pose2d tagPose2d = fieldLayout.getTagPose(12).get().toPose2d();

        double[] targetLocation = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
        if (targetLocation.length < 3) return;

        double tx = targetLocation[0];
        double tz = targetLocation[2];
        double yaw = LimelightHelpers.getTX(limelightName);
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        Pose2d visionPose;
        Pose2d aprilTagPose;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            visionPose = LimelightHelpers.getBotPose2d_wpiRed(limelightName);
            else visionPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);

        double distanceError = tz - Limelights.AprilTagLimits.ZError;
        double maxApproachSpeed = maxSpeed;

        if (Math.abs(distanceError) < 0.3) maxApproachSpeed = 0.5;
        if (Math.abs(distanceError) < 0.15) maxApproachSpeed = 0.25;

        SwerveRequest.FieldCentric newRequest;
        
        if (LimelightHelpers.getTV(limelightName)) {
            drivetrain.addVisionMeasurement(
                visionPose,
                Timer.getFPGATimestamp()
            );

            Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(
                (int)Math.round(LimelightHelpers.getFiducialID(limelightName))
            );

            if (tagPoseOpt.isEmpty()) return;

            aprilTagPose = tagPoseOpt.get().toPose2d();
            Pose2d relativePose = aprilTagPose.relativeTo(robotPose);

            double xError = relativePose.getX();
            double yError = relativePose.getY();

            double zOutput = zPID.calculate(xError, 0);
            double xOutput = xPID.calculate(yError, 0);
            xOutput = -Math.max(Math.min(xOutput, maxSpeed), -maxSpeed);
            zOutput = Math.max(Math.min(zOutput, maxApproachSpeed), -maxApproachSpeed);

            /*if (Math.abs(tz - Limelights.AprilTagLimits.ZError) < 0.1) {
                rotationOutput = turnPID.calculate(yaw, Limelights.AprilTagLimits.yawError);
                rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);
            }*/

            //debug
            System.out.println("tx: " + tx + " | xOutput:" + xOutput);
            System.out.println("tz: " + tz + " | zOutput:" + zOutput);
            System.out.println("yaw: " + yaw + " | rotOutput: " + 0);
            System.out.println("xpos: " + robotPose.getX());
            System.out.println("ypos: " + robotPose.getY());
            System.out.println("heading: " + robotPose.getRotation().getDegrees());

            newRequest = driveRequest.withVelocityX(-zOutput)
                                     .withVelocityY(xOutput)
                                     .withRotationalRate(0);
        } else {
            newRequest = driveRequest.withVelocityX(0.0)
                                     .withVelocityY(0.0)
                                     .withRotationalRate(0.0);
        }

        drivetrain.setControl(newRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0.0)
                                          .withVelocityY(0.0)
                                          .withRotationalRate(0.0));
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
