// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Volts;

// import com.ctre.phoenix6.configs.AudioConfigs;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// public class Hood extends SubsystemBase{
//     private final double GEAR_RATIO = 2;

//     private TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration()
//         .withAudio(new AudioConfigs()
//             .withBeepOnBoot(true)
//             .withBeepOnConfig(true))
//         .withCurrentLimits(new CurrentLimitsConfigs()
//             .withSupplyCurrentLimit(40)
//             .withSupplyCurrentLimitEnable(true))
//         .withMotorOutput(new MotorOutputConfigs()
//             .withInverted(InvertedValue.Clockwise_Positive)
//             .withNeutralMode(NeutralModeValue.Brake))
//         .withSlot0(new Slot0Configs()
//             .withKP(5)
//             .withKG(0.42)
//             .withKD(0.2)
//             .withGravityType(GravityTypeValue.Arm_Cosine))
//         .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
//             .withReverseSoftLimitThreshold(0)
//             .withReverseSoftLimitEnable(false)
//             .withForwardSoftLimitThreshold(0)
//             .withForwardSoftLimitEnable(false))
//         .withMotionMagic(new MotionMagicConfigs()
//             .withMotionMagicAcceleration(1));

//     private TalonFX hoodMotor = new TalonFX(25);

//     private SingleJointedArmSim hoodSim = new SingleJointedArmSim(
//         DCMotor.getKrakenX60(1), GEAR_RATIO, 0.5, Feet.of(1).in(Meters),
//         -3*Math.PI/2, 3*Math.PI/2, true, 0);

//     private double setpoint;
//     private final Trigger atSetpoint = new Trigger(() -> Math.abs(hoodMotor.getPosition().getValueAsDouble() - setpoint) < 0.01);

//     private Command goPlace = goToPos(0.5).withName("hood");

//     public Hood() {

//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putData(goPlace);
//         SmartDashboard.putNumber("2setpoint", setpoint);
//         SmartDashboard.putNumber("2pos", hoodMotor.getPosition().getValueAsDouble());
//     }

//     @Override
//     public void simulationPeriodic() {
//         var talonFXSim = hoodMotor.getSimState();

//         // set the supply voltage of the TalonFX
//         talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

//         // get the motor voltage of the TalonFX
//         var motorVoltage = talonFXSim.getMotorVoltageMeasure();

//         // use the motor voltage to calculate new position and velocity
//         // using WPILib's DCMotorSim class for physics simulation
//         hoodSim.setInputVoltage(motorVoltage.in(Volts));
//         hoodSim.update(0.020); // assume 20 ms loop time

//         // apply the new rotor position and velocity to the TalonFX;
//         // note that this is rotor position/velocity (before gear ratio), but
//         // DCMotorSim returns mechanism position/velocity (after gear ratio)

//         talonFXSim.setRawRotorPosition(hoodSim.getAngleRads()*GEAR_RATIO/(2*Math.PI));
//         talonFXSim.setRotorVelocity(hoodSim.getVelocityRadPerSec()*GEAR_RATIO/(2*Math.PI));
//     }

//     public Command set(double speed) {
//         return this.run(() -> hoodMotor.set(speed));
//     }

//     public Command goToPos(double pos) {
//         return this.run(() -> {
//             setpoint = pos;
//             hoodMotor.setControl(new MotionMagicExpoVoltage(pos));})
//             .until(atSetpoint);
//     }

//     public Trigger atSetpoint() {
//         return atSetpoint;
//     }

//     public double getPosition() {
//         return hoodMotor.getPosition().getValueAsDouble();
//     }

// }

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
    TalonFX armMotor = new TalonFX(9);
    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(0, 0, 0)//, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0, 0.865000, 50, 0))
        .withSoftLimit(Degrees.of(-30), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("hoodMOTOR", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        // .withClosedLoopRampRate(Seconds.of(0.25))
        // .withOpenLoopRampRate(Seconds.of(0.25))
        .withControlMode(ControlMode.CLOSED_LOOP);
    SmartMotorController smartMotorController = new TalonFXWrapper(armMotor,
                                                                    DCMotor.getKrakenX60(1),
                                                                    motorConfig);
    ArmConfig armCfg = new ArmConfig(smartMotorController)
        .withLength(Feet.of(3))
        .withMass(Pounds.of(3))
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withStartingPosition(Degrees.of(0)); // Parallel to the ground
        ;

    private Arm arm = new Arm(armCfg);

    private Command setAngle2 = setAngle(Degrees.of(90)).withName("Arm Angle 2");
    private Command setAngle1 = setAngle(Degrees.of(0)).withName("Arm Angle 1");

    public Hood() {
        // setDefaultCommand(setAngle1);
    }

    public Command setAngle(Angle angle) { return arm.setAngle(angle);}
    public Command set(double dutycycle) { return arm.set(dutycycle);}

    @Override
    public void periodic() {
        SmartDashboard.putData(setAngle2);
        SmartDashboard.putData(setAngle1);
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        arm.simIterate();
    }
}
