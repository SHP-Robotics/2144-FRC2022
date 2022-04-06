package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.kNominalVoltage;
import static frc.robot.Constants.kFalconSupplyCurrentConfig;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Drive extends SubsystemBase {
  private DifferentialDrive driveBase;

  // private final DifferentialDrivetrainSim drivetrainSim;

  public final WPI_TalonFX[] motors;
  // new WPI_TalonFX(0), // Left Front
  // new WPI_TalonFX(1), // Left Back
  // new WPI_TalonFX(2), // Right Front
  // new WPI_TalonFX(3) // Right Back

  private final MotorControllerGroup leftDrive;
  private final MotorControllerGroup rightDrive;

  private final AHRS navx;
  private double previousLinearAccelX;
  private double previousLinearAccelY;

  private final SimpleMotorFeedforward feedforward;

  // private double previousAngle = 0;

  // private final PIDController pid = new PIDController(0.01, 0, 0);

  private final DifferentialDriveOdometry odometry;
  private final DifferentialDriveKinematics kinematics;

  // simulation
  private TalonFXSimCollection leftFrontSim, leftBackSim, rightBackSim, rightFrontSim;
  private AnalogGyro gyro;
  private AnalogGyroSim gyroSim;
  private DifferentialDrivetrainSim driveSim;
  private final Field2d field;

  public Drive() {
    motors = new WPI_TalonFX[4];

    for (int i = 0; i < 4; i++) {
      motors[i] = new WPI_TalonFX(i);
      motors[i].configSupplyCurrentLimit(kFalconSupplyCurrentConfig);
      motors[i].setNeutralMode(NeutralMode.Coast);
    }

    this.enableRamp();

    leftDrive = new MotorControllerGroup(motors[0], motors[1]);
    rightDrive = new MotorControllerGroup(motors[2], motors[3]);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    driveBase = new DifferentialDrive(leftDrive, rightDrive);

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    navx = new AHRS(SPI.Port.kMXP);

    kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    if (RobotBase.isSimulation()) {
      gyro = new AnalogGyro(1);
      gyroSim = new AnalogGyroSim(gyro);
      leftFrontSim = motors[0].getSimCollection();
      leftBackSim = motors[1].getSimCollection();
      rightFrontSim = motors[2].getSimCollection();
      rightBackSim = motors[3].getSimCollection();
      driveSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2), // 2 Falcon 500 motors on each side of the drivetrain.
          kGearRatio,
          kMomentOfInertia,
          kMassKg,
          kWheelDiameterMeters / 2,
          kTrackWidthMeters,

          // The standard deviations for measurement noise:
          // x and y: 0.001 m
          // heading: 0.001 rad
          // l and r velocity: 0.1 m/s
          // l and r position: 0.005 m
          null// VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      );
    }

    odometry = RobotBase.isReal() ? new DifferentialDriveOdometry(navx.getRotation2d())
        : new DifferentialDriveOdometry(gyro.getRotation2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public boolean collisionDetected() {
    if (RobotBase.isSimulation())
      return false;
    double currentLinearAccelX = navx.getWorldLinearAccelX();
    double currentJerkX = currentLinearAccelX - previousLinearAccelX;
    previousLinearAccelX = currentLinearAccelX;

    double currentLinearAccelY = navx.getWorldLinearAccelY();
    double currentJerkY = currentLinearAccelY - previousLinearAccelY;
    previousLinearAccelY = currentLinearAccelY;

    return Math.abs(currentJerkX) > kCollisionThresholdDeltaG ||
        Math.abs(currentJerkY) > kCollisionThresholdDeltaG;
  }

  public void enableRamp() {
    for (WPI_TalonFX motor : motors)
      motor.configOpenloopRamp(0.8);
  }

  public void disableRamp() {
    for (WPI_TalonFX motor : motors)
      motor.configOpenloopRamp(0);
  }

  public void openLoop(double straight, double turn) {
    // prevent stick drift
    if (straight < 0.1 && straight > -0.1)
      straight = 0;
    if (turn < 0.1 && turn > -0.1)
      turn = 0;

    // if moving forward more than 20%, halve turn bias
    if (straight > 0.2)
      turn /= 2;

    // double driftCompensation = 0;
    // double angle = navx.getAngle();
    // if (rightX < 0.1 && rightX > -0.1) { // driving forward
    // rightX = 0;
    // driftCompensation = pid.calculate(angle, previousAngle);
    // } else previousAngle = angle; // turning
    // SmartDashboard.putNumber("drift compensation", driftCompensation);

    straight = MathUtil.clamp(straight, -kForwardThreshold, kForwardThreshold);
    boolean turnNegative = turn < 0;
    turn = MathUtil.clamp(Math.pow(turn, 2) * kTurningSensitivity, -kTurnThreshold, kTurnThreshold);
    if (turnNegative)
      turn = -turn;
    // rightX = Math.pow(rightX, 5) / Math.abs(Math.pow(rightX, 3));

    double leftSpeed = straight - turn;
    double rightSpeed = straight + turn;

    leftSpeed = MathUtil.clamp(leftSpeed, -kForwardThreshold, kForwardThreshold);
    rightSpeed = MathUtil.clamp(rightSpeed, -kForwardThreshold, kForwardThreshold);

    // System.out.println(leftSpeed);

    // System.out.println(rightSpeed);

    // driveBase.tankDrive(leftRamp.calculate(leftSpeed + driftCompensation),
    // rightRamp.calculate(rightSpeed));
    driveBase.tankDrive(leftSpeed, rightSpeed);
  }

  public void closedLoop(double leftMetersPerSecond, double rightMetersPerSecond) {
    leftDrive.setVoltage(feedforward.calculate(leftMetersPerSecond) * kNominalVoltage);
    leftDrive.setVoltage(feedforward.calculate(rightMetersPerSecond) * kNominalVoltage);
  }

  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public boolean isStopped() {
    return this.getChassisSpeeds().vxMetersPerSecond <= kSpeedThreshold;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(this.getWheelSpeeds());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // ticks/100ms * 10 = ticks/second
    double leftAverageTicksPerSecond = (motors[0].getSelectedSensorVelocity() + motors[1].getSelectedSensorVelocity())
        / 2 * 10;
    double rightAverageTicksPerSecond = (motors[2].getSelectedSensorVelocity() + motors[3].getSelectedSensorVelocity())
        / 2 * 10;
    return new DifferentialDriveWheelSpeeds(
        leftAverageTicksPerSecond * kMetersPerTick,
        rightAverageTicksPerSecond * kMetersPerTick);
  }

  public double getHeading() {
    return RobotBase.isReal() ? navx.getAngle() : gyro.getAngle();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    // driveBase.feed();
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Field2d getField() {
    return field;
  }

  public void resetEncoders() {
    for (WPI_TalonFX motor : motors)
      motor.setSelectedSensorPosition(0);
  }

  public void resetOdometry() {
    this.resetEncoders();
    odometry.resetPosition(new Pose2d(), navx.getRotation2d());
  }

  public void resetOdometry(Pose2d pose) {
    this.resetEncoders();
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void updateOdometry() {
    double leftAverageTicks = (motors[0].getSelectedSensorPosition() + motors[1].getSelectedSensorPosition()) / 2;
    double rightAverageTicks = (motors[2].getSelectedSensorPosition() + motors[3].getSelectedSensorPosition()) / 2;
    odometry.update(
        RobotBase.isReal() ? navx.getRotation2d() : gyro.getRotation2d(),
        leftAverageTicks * kMetersPerTick,
        rightAverageTicks * kMetersPerTick);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("navx angle", navx.getAngle());
    this.updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putBoolean("collision detected", collisionDetected());

    SmartDashboard.putNumber("navx pitch", navx.getPitch());
    SmartDashboard.putNumber("navx roll", navx.getRoll());
    SmartDashboard.putNumber("navx yaw", navx.getYaw());
    SmartDashboard.putNumber("navx angle", navx.getAngle());

    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(leftFrontSim.getMotorOutputLeadVoltage(),
        -rightFrontSim.getMotorOutputLeadVoltage());
    driveSim.update(0.02);

    // From NavX example
    // int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
    // "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    // angle.set(Math.IEEEremainder(-driveSim.getHeading().getDegrees(), 360));
    // navxSimAngle = -drivetrainSim.getHeading().getDegrees();

    updateSimEncoderPositions();
    updateSimEncoderVelocities();
    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  private void updateSimEncoderPositions() {
    int leftPosition = (int) (driveSim.getLeftPositionMeters() / kMetersPerTick);
    leftFrontSim.setIntegratedSensorRawPosition(leftPosition);
    leftBackSim.setIntegratedSensorRawPosition(leftPosition);

    int rightPosition = (int) (driveSim.getRightPositionMeters() / kMetersPerTick);
    rightFrontSim.setIntegratedSensorRawPosition(rightPosition);
    rightBackSim.setIntegratedSensorRawPosition(rightPosition);
  }

  private void updateSimEncoderVelocities() {
    int leftVelocity = (int) velocityToNativeUnits(
        driveSim.getLeftVelocityMetersPerSecond());
    leftFrontSim.setIntegratedSensorVelocity(leftVelocity);
    leftBackSim.setIntegratedSensorVelocity(leftVelocity);
    int rightVelocity = (int) velocityToNativeUnits(
        driveSim.getLeftVelocityMetersPerSecond());
    rightFrontSim.setIntegratedSensorVelocity(rightVelocity);
    rightBackSim.setIntegratedSensorVelocity(rightVelocity);
  }

  private double velocityToNativeUnits(double velocityMetersPerSecond) {
    return velocityMetersPerSecond * kVelMetersToEnc;
  }
}
