package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    public enum IntakeStates {
        ON(5),
        CYCLE(2),
        OFF(0),
        REVERSE(-2);

        double voltage;
        private IntakeStates(double voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {
            return voltage;
        }
    }

    private TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(Constants.HardwarePorts.intake, "mechbussy"); //get real port

        configureMotor(intakeMotor, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    }

    private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = direction;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.intakeStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
        
        motor.optimizeBusUtilization();
    }

    public void setVoltage(double voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public Command setState(IntakeStates state){
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    @Override
    public void periodic() {

    }


    // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(
    //     Volts.of(1).div(Seconds.of(1)),
    //     Volts.of(8),
    //     Units.Seconds.of(8),
    //     (state) -> SignalLogger.writeString("SysIdIntake_State", state.toString())
    // ),
    // new SysIdRoutine.Mechanism(
    //     (volts) -> setVoltage(volts.in(Volts)),
    //     log -> {log.motor("intake_motor")
    //         .voltage(intakeMotor.getMotorVoltage().getValue())
    //         .angularPosition(intakeMotor.getPosition().getValue())
    //         .angularVelocity(intakeMotor.getVelocity().getValue()); },
    //     this
    // ));

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.dynamic(direction);
    // }

}
