package frc.robot;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.PoundInches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {
    private SparkMax raw_shooter_motor0 = new SparkMax(51, MotorType.kBrushless);
    //private SparkMax raw_shooter_motor1 = new SparkMax(52, MotorType.kBrushless);


    private SmartMotorControllerConfig shooter_motor_config = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withSimClosedLoopController(0.1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 1, 0))

        .withGearing(1)
        .withStatorCurrentLimit(Amps.of(40))
        // .withSupplyCurrentLimit(Amps.of(40))
        .withIdleMode(MotorMode.COAST)

        .withMotorInverted(false)

        .withTelemetry("MOTORshooter", TelemetryVerbosity.HIGH)

        .withFollowers(Pair.of(new SparkMax(42, MotorType.kBrushless), true))
        ;

    private SmartMotorController smc = new SparkWrapper(raw_shooter_motor0, DCMotor.getNEO(2), shooter_motor_config);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
        // Diameter of the flywheel.
        // .withDiameter(Inches.of(4.95))
        // Mass of the flywheel.
        // .withMass(Pounds.of(2.54))

        .withMOI(MomentOfInertia.ofRelativeUnits(6.5, Pounds.mult(InchesPerSecond).mult(Inches).per(RadiansPerSecond)))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("MECHshooter", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    private FlyWheel shootermech = new FlyWheel(shooterConfig);


    public Shooter() {

    }

    public Command setVelocity(AngularVelocity velocity) {
        return shootermech.setSpeed(velocity);
    }

    public AngularVelocity getVelocity() {
        return AngularVelocity.ofBaseUnits(shootermech.getSpeed().in(RPM), RPM);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Not a bad word", getVelocity().baseUnitMagnitude());
        shootermech.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shootermech.simIterate();
    }
}
