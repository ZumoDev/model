// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Indexer;
import frc.robot.commands.IntakePos;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.WristPos;
import frc.robot.commands.WristSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CANManager;
import frc.robot.utils.PowerManager;
import frc.robot.utils.RobotDiagnostics;
import frc.robot.commands.ATTRotation;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.AprilTags.BlueAlliance;
import frc.robot.Constants.AprilTags.RedAlliance;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
    private final CANManager m_canManager = new CANManager();
    private final PowerManager m_powerManager = new PowerManager();
    private final RobotDiagnostics m_errorManager = new RobotDiagnostics();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(0);
    private final IntakeSubsystem intakeSub = new IntakeSubsystem();
    private final ShooterSubsystem shooterSub = new ShooterSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final String limelightName = Constants.Limelights.shooterLimelightName;
    private static boolean isCalculationEnabled = true;

    private final PIDController rotationPID = new PIDController(
        Constants.Limelights.PID.Rotation.kP,
        Constants.Limelights.PID.Rotation.kI,
        Constants.Limelights.PID.Rotation.kD
    );

        private final SendableChooser<Command> autoChooser;


    public static final List<Integer> redAllianceTags = List.of(
        RedAlliance.bottomHubId, RedAlliance.bottomSideHubId,
        RedAlliance.topHubId, RedAlliance.topSideHubId,
        RedAlliance.frontHubId, RedAlliance.frontSideHubId
    );

    public static final List<Integer> blueAllianceTags = List.of(
        BlueAlliance.bottomHubId, BlueAlliance.bottomSideHubId,
        BlueAlliance.topHubId, BlueAlliance.topSideHubId,
        BlueAlliance.frontHubId, BlueAlliance.frontSideHubId
    );

    public RobotContainer() {

 // ===== NAMED COMMANDS =====

NamedCommands.registerCommand(
    "IntakeUp",
    new IntakePos(intakeSub, 0).withTimeout(1)
);

NamedCommands.registerCommand(
    "IntakeOn",
    new StartIntake(intakeSub, 1).withTimeout(2.0)
);

NamedCommands.registerCommand(
    "StartShooter",
    Commands.parallel(
        new StartShooter(shooterSub, intakeSub, 1, false, () -> null),
        new Indexer(intakeSub,-0.5)
    ).withTimeout(2.0)
);

NamedCommands.registerCommand(
    "ShooterInitial",
    Commands.parallel(
        new StartShooter(shooterSub, intakeSub, 1, false, () -> null),
        Commands.sequence(Commands.waitSeconds(0.5), new Indexer(intakeSub, -0.5).withTimeout(2.0)))
);

NamedCommands.registerCommand(
    "ShooterOn",
     Commands.parallel(
        new StartShooter(shooterSub, intakeSub, 1, false, () -> null),
        Commands.sequence(Commands.waitSeconds(0.5), new Indexer(intakeSub, -0.5).withTimeout(2.0)))
);

NamedCommands.registerCommand(
    "IntakeDownUpDown",
    Commands.sequence(
        new IntakePos(intakeSub, 3.25).withTimeout(0.2),
        new IntakePos(intakeSub, 0).withTimeout(0.2),
        new IntakePos(intakeSub, 3.25).withTimeout(0.2)
    )
);
NamedCommands.registerCommand("IntakeDown", new IntakePos(intakeSub, 3.25).withTimeout(1.0));

          autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);

        //joystick 1
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.rightTrigger().whileTrue(new StartIntake(intakeSub, 0.9));   // absorber
        joystick.leftTrigger().whileTrue(new StartIntake(intakeSub, -0.75));   // expulsar
        joystick.rightBumper().onTrue(new IntakePos(intakeSub, 3.5));            // bajar (recolectar)
        joystick.leftBumper().onTrue(new IntakePos(intakeSub, 0));             // subir (home)
        joystick.x().whileTrue(new Indexer(intakeSub, 0.5));
        joystick.b().whileTrue(new Indexer(intakeSub, -0.5));

        //
        joystick2.rightTrigger().whileTrue(Commands.parallel(
            new StartShooter(shooterSub, intakeSub, 0.75, isCalculationEnabled, this::getClosestAprilTag),
            new ATTRotation(drivetrain, drive, rotationPID, MaxAngularRate, limelightName)
                .onlyIf(() -> isCalculationEnabled)
        ));

        joystick2.leftTrigger().whileTrue(new StartIntake(intakeSub, 0.75));

        //movimiento en circunferencia...
        joystick2.rightBumper().onTrue(
            Commands.runOnce(() -> isCalculationEnabled = !isCalculationEnabled)
        );
    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }

    public void updateCANTelemetry() {
        m_canManager.update();
    }

    public void updatePowerTelemetry() {
    m_powerManager.update();

    }
    public void updateTelemetry() {
        m_errorManager.update();
    }

    public void logRobotError(String msg) {
        m_errorManager.addLog(msg, true);
    }
    
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }
        
        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public LimelightHelpers.LimelightTarget_Fiducial getClosestAprilTag() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        LimelightHelpers.LimelightTarget_Fiducial closestTag = null;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        double closestDistance = Double.MAX_VALUE;

        if (results != null && results.targets_Fiducials != null) {
            for (LimelightHelpers.LimelightTarget_Fiducial tag : results.targets_Fiducials) {
                if (tag != null) {
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        if (!redAllianceTags.contains((int)tag.fiducialID)) continue;
                        else break;
                    } else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
                        if (!blueAllianceTags.contains((int)tag.fiducialID)) continue;
                        else break;
                    }

                    Pose3d targetLocation = tag.getTargetPose_CameraSpace();
                    Translation3d translation = targetLocation.getTranslation();

                    double distance = translation.getNorm(); 

                    if (distance < closestDistance) {
                        closestTag = tag;
                        closestDistance = distance;
                    };
                }
            }
        }

        return closestTag;
    }

    public void updateVisionDashboard() {
        // 1. Obtener la pose del objetivo relativa a la cámara (Camera Space)
        // El arreglo contiene: [x, y, z, roll, pitch, yaw]
        double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace("limelight");

        // 2. Validación de seguridad (si no hay target, el arreglo suele estar vacío o en 0)
        // tv es 1 si la Limelight ve un objetivo
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (hasTarget && targetPose.length >= 3) {
            // Extraemos las coordenadas 3D (en metros)
            double x = targetPose[0]; // Desplazamiento lateral (izquierda/derecha)
            double y = targetPose[1]; // Desplazamiento vertical (arriba/abajo)
            double z = targetPose[2]; // Profundidad (hacia adelante)

            // 3. LA FÓRMULA MAESTRA: Distancia Euclidiana 3D
            // Calcula la línea recta real desde el lente hasta el AprilTag
            double distance = Math.sqrt(
                Math.pow(x, 2) + 
                Math.pow(y, 2) + 
                Math.pow(z, 2)
            );

            // 4. Publicar al Widget (Llave: "TargetDistance")
            SmartDashboard.putNumber("TargetDistance", distance);
            
            // Opcional: Publicar TX para que el radar del widget se mueva
            SmartDashboard.putNumber("TargetTX", LimelightHelpers.getTX("limelight"));
            
        } else {
            // Si no hay target, mandamos 0 para que el widget muestre "---"
            SmartDashboard.putNumber("TargetDistance", 0.0);
        }
    }

    //METODO PARA EL CAN ANCHO DE BANDA, RIO TEMP
    public void updateSystemHealth() {
        // 1. CAN Bus Utilization
        var canStatus = RobotController.getCANStatus();
        // Enviamos decimal (0.0 a 1.0) porque tu widget hace (canUsage * 100)
        double canUsage = canStatus.percentBusUtilization / 100.0; 
        // En 2026 intentaron separar la carga del sistema de la carga del proceso
        double rioTemp = RobotController.getCPUTemp();

        // PUBLICACIÓN
        SmartDashboard.putNumber("CANUsage", canUsage);
        SmartDashboard.putNumber("RoboRIOTemp", rioTemp);
    }
}
