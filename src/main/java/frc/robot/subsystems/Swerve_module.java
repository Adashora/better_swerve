// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve_module {
    public SparkMax drive_motor;
    public SparkMax turn_motor; // defining variables
    public int module_number;

    public double turn_speed;
    public double drive_speed;
    PIDController turn_PID;
    PIDController drive_PID;

    Rotation2d turn_offset;

    private final CANCoder best_turnEncoder;
    private AbsoluteSensorRange range;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;

    private final SparkMaxConfig drive_config;
    private final SparkMaxConfig turn_config;

    public Swerve_module(int module_number, int drive_id, int turn_motor_ID, int cancoder_ID,
            Rotation2d turn_offset) {
        this.turn_offset = turn_offset;
        this.drive_config = new SparkMaxConfig();
        this.turn_config = new SparkMaxConfig();

        this.module_number = module_number;

        // cancoder stuff
        this.best_turnEncoder = new CANCoder(cancoder_ID);
        range = AbsoluteSensorRange.Signed_PlusMinus180;
        this.best_turnEncoder.configAbsoluteSensorRange(range); // sets the range of the encoder

        // drive motors stuff
        this.drive_motor = new SparkMax(drive_id, MotorType.kBrushless); // defines drive motor
        this.drive_encoder = this.drive_motor.getEncoder(); // gerts the encoder fomr the dirve motor

        drive_config
                .voltageCompensation(12) // configs for drive
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        drive_config.encoder // configs for encoder
                .positionConversionFactor(Constants.position_conversion_factor)
                .velocityConversionFactor(Constants.velocity_conversion_factor);
        // sets the current limit to 40 amps

        this.drive_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drive_PID = new PIDController(Constants.drivekp, Constants.driveki, Constants.drivekd); //

        // turn motor stuff
        turn_motor = new SparkMax(turn_motor_ID, MotorType.kBrushless); // defines turn motor
        this.turn_encoder = this.turn_motor.getEncoder(); // gets the encoder from the turn motor

        best_turnEncoder.configAbsoluteSensorRange(range);

        turn_config
            .voltageCompensation(12)
            .smartCurrentLimit(20) // limits the current to 20 amps
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        turn_config.encoder
            .positionConversionFactor(Constants.turn_pos_factor); // converts the position to degrees
        turn_config.signals
            .primaryEncoderPositionPeriodMs(500); // sets the update period of the encoder to 500 ms

        this.turn_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turn_PID = new PIDController(Constants.turnKP, Constants.turnKI, Constants.turnKD);

    }

    public Rotation2d getCANCoder() { // gets the angle of the cancoder
        double angle = this.best_turnEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Cancoder angel" + this.module_number, angle);
        return Rotation2d.fromDegrees(angle);
    }

    public SwerveModuleState getState() { // gets the state of the module
        double speedMetersPerSecond = this.drive_encoder.getVelocity(); // * Constants.dt.velocity_conversion_factor;
        Rotation2d angle = getCANCoder();
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public void set_desired_state(SwerveModuleState desiredState) { // sets the desired state of the module
        SwerveModuleState current_state = this.getState();

        Rotation2d current_rotation = current_state.angle.minus(this.turn_offset).plus(new Rotation2d()); /* gets the
                                                                                                             current
                                                                                                             rotation of
                                                                                                             the module */

        desiredState.optimize(current_rotation);

        desiredState.cosineScale(current_rotation); /* scales the desired state by the cosine of the difference between
                                                    the desired state and the current state
                                                    if angle dif is small then motor power is faster
                                                    if angle dif is big then motor power is slower I think */

        Rotation2d desired_rotation = desiredState.angle.plus(new Rotation2d());

        double diff = desired_rotation.getDegrees() - current_rotation.getDegrees(); /*gets the difference between the
                                                                                       desired rotation and the current
                                                                                       rotation */

        if (Math.abs(diff) < 1) { // if the difference is less than 1 then the turn speed is 0
            turn_speed = 0;
        } else { // otherwise the turn speed is calculated by the PID controller
            turn_speed = turn_PID.calculate(diff, 0);
        }

        drive_speed = desiredState.speedMetersPerSecond / Constants.max_speed;

        turn_motor.set(turn_speed); // sets the turn speed
        drive_motor.set(drive_speed); // sets the drive speed
        SmartDashboard.putNumber("Desired rotaiton" + this.module_number, desired_rotation.getDegrees());
        SmartDashboard.putNumber("Current Rotation" + this.module_number, current_rotation.getDegrees());
    }

}
