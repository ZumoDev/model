// Paquete
package frc.robot.subsystems;

// Importaciones
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

// Subsistema de la intake y el indexer del robot
public class IntakeSubsystem extends SubsystemBase {

    // Motores de la intake (NEO brushless vía SPARK MAX, ecosistema REV)
    public SparkMax intakeMotorLeft, intakeMotorRight;

    // Controlador de lazo cerrado y encoder del motor izquierdo (líder)
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    // Motor de rodillos de la intake (Kraken X44 TalonFX, ecosistema CTRE)
    public TalonFX rollerMotor;

    // Motor del indexer (TalonFX, ecosistema CTRE)
    public TalonFX indexerMotor;

    // Modos de control para el indexer
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);

    // Constructor
    public IntakeSubsystem() {
        // Motores de la intake (SPARK MAX con NEO)
        intakeMotorLeft  = new SparkMax(Ports.Intake.LEFT_SPARK_ID,  MotorType.kBrushless);
        intakeMotorRight = new SparkMax(Ports.Intake.RIGHT_SPARK_ID, MotorType.kBrushless);

        // Configurar motor izquierdo (líder) con PID de posición en Slot 0
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.2, ClosedLoopSlot.kSlot0)
            .i(0.0, ClosedLoopSlot.kSlot0)
            .d(0.0, ClosedLoopSlot.kSlot0);
        intakeMotorLeft.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Motor derecho sigue al izquierdo (invertido para giro opuesto)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(intakeMotorLeft, true);
        intakeMotorRight.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Obtener referencias al controlador PID y encoder del líder
        closedLoopController = intakeMotorLeft.getClosedLoopController();
        encoder = intakeMotorLeft.getEncoder();

        // Rodillos de la intake (Kraken X44 TalonFX en bus RIO)
        rollerMotor = new TalonFX(Ports.Intake.ROLLER_ID, Ports.RIO_BUS);
        rollerMotor.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                )
        );

        // Indexer (TalonFX en bus RIO)
        indexerMotor = new TalonFX(Ports.INDEXER_ID, Ports.RIO_BUS);
        configurarIndexer(indexerMotor, InvertedValue.Clockwise_Positive);
    }

    // Configuración del TalonFX del indexer
    private void configurarIndexer(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond))
            );

        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/EncoderPos", getPosition());
    }

    // Detiene todos los motores: brazo (líder SparkMax) y rodillo (TalonFX)
    public void stop() {
        intakeMotorLeft.set(0);
        rollerMotor.set(0);
    }

    // Control por voltaje porcentual para un motor TalonFX específico
    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    // Control por RPM con lazo PID (Slot0) para un motor TalonFX específico
    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    // Velocidad del rodillo Kraken X44 (rango -1 a 1). Solo controla el TalonFX de rollers.
    // Llamar desde triggers del driver para absorber (+) o expulsar (-).
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    // Velocidad directa del brazo (NEO SparkMax líder, rango -1 a 1).
    // Usar para torque de sostenimiento al finalizar IntakePos, NO para los rodillos.
    public void setArmSpeed(double speed) {
        intakeMotorLeft.set(speed);
    }

    public void setWristSpeed(double speed) {}

    // Posición 0 = home (arriba), posición 4 = recolección (abajo)
    public void setPosition(double position) {
        closedLoopController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    // Velocidad directa del indexer (rango -1 a 1)
    // TODO: velocidad del feeder para alimentar el shooter — calibrar segun nota (actualmente Constants.Shooter.FEEDER_SPEED)
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
