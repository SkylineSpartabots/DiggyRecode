package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake.IntakeStates;

public class Pivot extends SubsystemBase {
    private static Pivot instance;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public static Pivot getInstance() {
        if(instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public enum PivotStates {
        HARD_DEPLOY(3.5),
        DEPLOY(2.2), 
        RETRACT(-2.5),
        OFF(0);

        double voltage;
        private PivotStates(double voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {
            return voltage;
        }   
    }

    private TalonFX pivotMotor;
    // private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);

    public Pivot() {
        pivotMotor = new TalonFX(Constants.HardwarePorts.pivot, "mechbussy"); //get real port
        
        configureMotor(pivotMotor, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    }

    private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // config.Slot0.kP = 0.5;
        // // config.Slot0.kD = 0.001;
        // config.Slot0.kG = 0.5;

        config.MotorOutput.Inverted = direction;
        config.MotorOutput.NeutralMode = neutralMode;

        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.pivotSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.pivotStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        motor.getPosition().setUpdateFrequency(150);
        motor.getStatorCurrent().setUpdateFrequency(150);
        motor.getSupplyCurrent().setUpdateFrequency(150);
        motor.optimizeBusUtilization();
    }

    // public void setRotations(double rotations) {
    //     pivotMotor.setControl(mmRequest.withPosition(rotations));
    // }

    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageRequest.withOutput(volts));
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public Command setState(PivotStates state){
        return Commands.runOnce(() -> setVoltage(state.getVoltage()), this);
    }

    // public void zeroPivot() {
    //     pivotMotor.setPosition(0);
    // }

    // public double getPosition() {
    //     return pivotMotor.getPosition().getValueAsDouble();
    // }

    public StatusSignal<Current> getCurrent() {
        return pivotMotor.getStatorCurrent();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("pivot pos", getPosition());
    }
}
