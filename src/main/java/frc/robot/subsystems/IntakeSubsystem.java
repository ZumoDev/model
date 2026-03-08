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

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//Create class
public class IntakeSubsystem extends SubsystemBase {

    //Atributes
    public TalonFX intakeMotor, indexerMotor;
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);
    private final PositionVoltage posvol = new PositionVoltage(0);

    //Constructor
    public IntakeSubsystem() {
        //liMotor = new TalonFX(0); //sparj
        //riMotor = new TalonFX(0); //spark
        intakeMotor = new TalonFX(3);
        indexerMotor = new TalonFX(4);

        //configureMotor(liMotor, InvertedValue.CounterClockwise_Positive);
        //configureMotor(riMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(intakeMotor, InvertedValue.Clockwise_Positive);
        configureMotor(indexerMotor, InvertedValue.Clockwise_Positive);
        //liMotor.setControl(new Follower(riMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    //Metodo pra publicar el vlaor mas alto de la tempertarua del intake
    /*public double getIntakeTemp(){
        double templiMotor= liMotor.getDeviceTemp().getValueAsDouble();
        double tempriMotor=riMotor.getDeviceTemp().getValueAsDouble();
        double tempIntakeMotor=riMotor.getDeviceTemp().getValueAsDouble();
        double tempIndexermotor=riMotor.getDeviceTemp().getValueAsDouble();
        return Math.max(Math.max(tempIntakeMotor, tempIndexermotor),Math.max(templiMotor, tempriMotor));
    }*/

    //Si sucede algun error, es porque el idiota de zumo le puso la misma config a todos los motores...
    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
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
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setWristSpeed(double speed) {
        //riMotor.set(speed);
    }

    public void setPosition(double position) {
        //riMotor.setControl(posvol.withPosition(position));
    }

    public double getPosition() {
        return 0; //riMotor.getPosition().getValueAsDouble();
    }

    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
