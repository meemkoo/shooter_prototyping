package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {
    // public static enum States {
    //     idle(false, RPM.of(0)),
    //     spinUp(false, RPM.of(2000))
    //     ;

    //     public Boolean kickerRunning;
    //     public AngularVelocity flywheelVelocity;

    //     States(Boolean kickerRunning, AngularVelocity flywheelVelocity) {
    //         this.kickerRunning = kickerRunning;
    //         this.flywheelVelocity = flywheelVelocity;
    //     }
    // }


    private static class FlywheelConstants {
        private static double kP = 0.0001;
        private static double kI = 0.0;
        private static double kD = 0.0;

        private static double kS = 0.0;
        private static double kV = 0.12;
        private static double kA = 0.0;

        private static MechanismGearing gearbox = new MechanismGearing(1);
        private static DCMotor motors = DCMotor.getNEO(2);
        private static MomentOfInertia MOI = KilogramSquareMeters.of(0.0019021577);

        private static ExponentialProfilePIDController exprofpidctrl = new ExponentialProfilePIDController(
            kP, kI, kD, 
            ExponentialProfilePIDController.createFlywheelConstraints(
                Volts.of(5),
                motors, MOI, gearbox)
        );

        private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }


    // Motor Configs

    private SparkMax rawKickerMotor = new SparkMax(51, MotorType.kBrushless);
    private SparkMax rawFlywheelMotor0 = new SparkMax(52, MotorType.kBrushless);
    private SparkMax rawFlywheelMotor1 = new SparkMax(53, MotorType.kBrushless);

    private SmartMotorControllerConfig flywheelMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
            // .withClosedLoopController(0.000100, 0, 0)//, RotationsPerSecond.of(6000), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(FlywheelConstants.exprofpidctrl)//, RotationsPerSecond.of(6000), DegreesPerSecondPerSecond.of(45))
            .withClosedLoopController(FlywheelConstants.exprofpidctrl)

            .withFeedforward(FlywheelConstants.feedforward)
            .withSimFeedforward(FlywheelConstants.feedforward)
        .withGearing(FlywheelConstants.gearbox)
        .withIdleMode(MotorMode.COAST)
        .withMotorInverted(false)

        .withTelemetry("flywheelMOTOR", TelemetryVerbosity.HIGH)
        .withFollowers(Pair.of(rawFlywheelMotor1, true)) // The other motor, inverted
        .withStatorCurrentLimit(Amps.of(60))
        ;


    // Motors

    private SmartMotorController kickerMotor = new SparkWrapper(rawKickerMotor, DCMotor.getNEO(1),
        // Config is tiny, so we dont use a seprate config
        new SmartMotorControllerConfig(this)
            .withGearing(new MechanismGearing(2, 1))
            .withControlMode(ControlMode.OPEN_LOOP)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("kickerMOTOR", TelemetryVerbosity.MID)

            .withMotorInverted(true)
    );

    private SmartMotorController flywheelMotor = new SparkWrapper(rawFlywheelMotor0, DCMotor.getNEO(2), flywheelMotorConfig);

    // Mechanisim (Alex Reference)
    FlyWheelConfig flywheelConfig = new FlyWheelConfig(flywheelMotor)
        .withMOI(KilogramSquareMeters.of(0.0019021577))
        .withUpperSoftLimit(RPM.of(6000))
        .withTelemetry("flywheelMECH", TelemetryVerbosity.HIGH);


    private FlyWheel flywheel = new FlyWheel(flywheelConfig);

    // Other
    private AngularVelocity flywheelsetpoint = RPM.of(0);
    private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> flywheelsetpoint;

    // Triggers

    // private Debouncer flywheelSetpointDebounce = new Debouncer(1, Debouncer.DebounceType.kBoth);
    // private BooleanSupplier flywheelAtSetpoint = 
    //     () -> flywheelSetpointDebounce.calculate( 
    //         Math.abs(flywheel.getSpeed().in(RPM)-flywheelsetpoint.in(RPM)) < 70
    //     );

    // Commands

    // private Command spinUpCmd = Commands.repeatingSequence(flywheel.setSpeed(RPM.of(2000)))
    //     // .until(flywheelAtSetpoint)
    //     .withName("spinUp");
    // private Command spinDownCmd = Commands.repeatingSequence(flywheel.setSpeed(RPM.of(0)))
    //     // .until(flywheelAtSetpoint)
    //     .withName("spinDown");

    // private Command kickOnceCmd = run(() -> kickerMotor.setDutyCycle(1))
    //     .withTimeout(Seconds.of(0.5))
    //     .andThen(runOnce(() -> kickerMotor.setDutyCycle(0)))
    //     .withName("Kick");

    private Command startShooting;

    public Shooter() {
        startShooting = startShooting();
        SmartDashboard.putNumber("flysetpoint", 0);
        // setDefaultCommand(flywheel.setSpeed(RPM.of(0)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(startShooting);
        // SmartDashboard.putData(kickOnceCmd);
        // SmartDashboard.putData(startShooting);
        // SmartDashboard.putData(spinUpCmd);
        // SmartDashboard.putBoolean("At Setpoint", flywheelAtSetpoint.getAsBoolean());
        flywheelsetpoint = RPM.of(SmartDashboard.getNumber("flysetpoint", 0));
        SmartDashboard.putNumber("RPM", flywheel.getSpeed().in(RPM));

        Optional<AngularVelocity> setvelocity = flywheelMotor.getMechanismSetpointVelocity();

        if (setvelocity.isEmpty()) {
            SmartDashboard.putNumber("yetpoint", 0);
            // SmartDashboard.putNumber("minus", 0);
        } else { 
            // SmartDashboard.putNumber("minus", flywheel.getSpeed().in(RPM)-setvelocity.get().in(RPM));
            SmartDashboard.putNumber("yetpoint", setvelocity.get().in(RPM));
        }
        flywheel.updateTelemetry();
        kickerMotor.updateTelemetry();
    }

    public Command startShooting() {
        // return Commands.run(() -> {

        // }).alongWith(flywheel.setSpeed(flywheelVelocitySupplier))
        return flywheel.setSpeed(flywheelVelocitySupplier)
        .withName("start shooting")
        ;
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
        kickerMotor.simIterate();
    }
}
