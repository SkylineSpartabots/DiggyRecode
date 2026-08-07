package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private static Indexer instance;

    public static Indexer getInstance() {
        if(instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    public enum IndexerStates{
        ON(7.5),
        OFF(0),
        REVERSE(-3);

        double voltage;
        private IndexerStates(double speed) {
            this.voltage = speed;
        }

        public double getVoltage() {
            return voltage;
        }
    }

    private TalonFX indexerMotor;

    public Indexer() {
        indexerMotor = new TalonFX(Constants.HardwarePorts.indexer, "mechbussy"); //get real port


        config(indexerMotor, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    }

    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;


        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.indexerStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);


        // 20 ms
        motor.getVelocity().setUpdateFrequency(50);
        
        motor.getConfigurator().apply(config);

        motor.optimizeBusUtilization();
    }

    public void setVoltage(double voltage) {
        indexerMotor.setControl(new VoltageOut(voltage));
    }

    public Command setState(IndexerStates state){
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    @Override
    public void periodic() {

    }
}
