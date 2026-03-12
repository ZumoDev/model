//Package
package frc.robot.subsystems;

//Imports
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


//Create class
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax leader   = new SparkMax(Ports.Intake.RIGHT_SPARK_ID, MotorType.kBrushless);
    private final SparkMax follower = new SparkMax(Ports.Intake.LEFT_SPARK_ID, MotorType.kBrushless);
    private final TalonFX  rollers  = new TalonFX(Ports.Intake.ROLLER_ID, Ports.RIO_BUS);
    private final TalonFX  indexer  = new TalonFX(Ports.INDEXER_ID, Ports.RIO_BUS);

    private final RelativeEncoder           encoder;
    private final SparkClosedLoopController pid;

    // ── Positions (tunable) ──────────────────────────────────
    public static final double ARM_UP      = 3.2;   // holds against gravity
    public static final double ARM_DOWN    = 0;   // collect position
    public static final double ARM_SHOOT_A = 3.2;   // firing oscillation A
    public static final double ARM_SHOOT_B = 3.1;   // firing oscillation B

    public IntakeSubsystem() {
        SparkMaxConfig leaderCfg = new SparkMaxConfig();
        leaderCfg.inverted(false);
        leaderCfg.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(3.4)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0);
        leaderCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5, ClosedLoopSlot.kSlot0)
            .i(0,   ClosedLoopSlot.kSlot0)
            .d(0.05,   ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        leader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerCfg = new SparkMaxConfig();
        followerCfg.follow(leader, true); // inverted follow
        follower.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();
        pid     = leader.getClosedLoopController();

        // Indexer — CounterClockwise = toward shooter
        TalonFXConfiguration idxCfg = new TalonFXConfiguration();
        idxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexer.getConfigurator().apply(idxCfg);

        // Rollers — adjust inversion if needed after test
        TalonFXConfiguration rolCfg = new TalonFXConfiguration();
        rolCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollers.getConfigurator().apply(rolCfg);
    }

    // ── Public API ────────────────────────────────────────────
    public void setArmPosition(double pos) {
        pid.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setRollerSpeed(double speed) {
        rollers.set(speed);
    }

    public void setIndexerSpeed(double speed) {
        indexer.set(speed);
    }

    public double getArmPosition() {
        return encoder.getPosition();
    }

    public void stopAll() {
        rollers.stopMotor();
        indexer.stopMotor();
    }
}
