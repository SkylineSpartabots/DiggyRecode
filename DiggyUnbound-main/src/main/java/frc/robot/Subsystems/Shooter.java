package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.HardwarePorts;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    public static Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private TalonFX topL_leader, botL, topR, botR;

    private final VelocityVoltage rpsRequest = new VelocityVoltage(0).withSlot(0);
    VoltageOut voltageRequest = new VoltageOut(0);

    private double airtime; // seconds

    public Shooter() {
        topL_leader = new TalonFX(HardwarePorts.shooterTL, "mechbussy");
        botL = new TalonFX(HardwarePorts.shooterBL, "mechbussy");
        topR = new TalonFX(HardwarePorts.shooterTR, "mechbussy");

        config(topL_leader, NeutralModeValue.Coast, InvertedValue.CounterClockwise_Positive);
        config(botL, NeutralModeValue.Coast, InvertedValue.CounterClockwise_Positive);
        config(topR, NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);

        botL.setControl(new Follower(topL_leader.getDeviceID(), MotorAlignmentValue.Aligned));
        topR.setControl(new Follower(topL_leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        // sysid
        config.Slot0.kS = 0.3004;
        config.Slot0.kV = 0.11613;
        config.Slot0.kA = 0.02206;
        config.Slot0.kP = 0.047173;
        config.Slot0.kD = 0;

        // 20 ms
        motor.getVelocity().setUpdateFrequency(75);
        motor.getPosition().setUpdateFrequency(75);

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.shooterSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.shooterStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);  
        
        motor.optimizeBusUtilization();
    }

    // returns rps
    public double getLeaderVelocity() {
        return topL_leader.getVelocity().getValueAsDouble();
    }
    
    public double[] getAllVelocities() {
        return new double[] {
            topL_leader.getVelocity().getValueAsDouble(),
            botL.getVelocity().getValueAsDouble(),
            topR.getVelocity().getValueAsDouble(),
            botR.getVelocity().getValueAsDouble()
        };
    }
    
    /**
     * set velocity of motors
     * @param velocity in rps
    */
    public void setVelocity(double velocity) {
        if (velocity == 0)
            topL_leader.setControl(voltageRequest.withOutput(0));
        else
            topL_leader.setControl(rpsRequest.withVelocity(velocity));
    }

    /**
     * set velocity of motors
     * @param voltage
    */
    public void setVoltage(double voltage) {
        topL_leader.setControl(voltageRequest.withOutput(voltage));
    }

    public double loggerProEvilEquationOfDoom(double d) {
        System.out.println(d);
        return (7.2 * d + 25.8);
    }

    public void updateAirtime(double airtime) {
        this.airtime = airtime;
    }

    public double getAirtime() {
        return airtime;
    }

    //sysid

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
        Volts.of(1).div(Seconds.of(1)),
        Volts.of(8),
        Units.Seconds.of(8),
        (state) -> SignalLogger.writeString("SysIdShooter_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
        (volts) -> setVoltage(volts.in(Volts)),
        log -> {log.motor("shooter_leader")
            .voltage(topL_leader.getMotorVoltage().getValue())
            .angularPosition(topL_leader.getPosition().getValue())
            .angularVelocity(topL_leader.getVelocity().getValue()); },
        this
    ));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}