package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.SwerveUtilLOP;

/**
 * Put brief class description here...
 */
public class SwerveSystem {
    // final static double SV_RELIC_GRABBER_INIT = 0.5039;
    public boolean use_verbose = false;
    public boolean use_swerve = false;   // use four motors and four servos for chassis
    //public boolean use_newbot = false;   // use four motors and four servos for new chassis
    //public boolean use_newbot_v2 = true;
    public boolean use_front_drive_only = false;
    public boolean use_imu = true;
    public boolean use_imu2 = false;
    public boolean use_range_sensor = false;
    public boolean use_proximity_sensor = false;
    public boolean use_encoder = true;
    boolean stop_on_bump = false; // [autonomous variable]
    boolean bump_detected = false; // [autonomous variable]

    public boolean fast_mode = true; //Controls how "rushed" autonomous actions are [autonomous variable]
    public boolean straight_mode = false; // [cart variable]
    public boolean deliver_mode = false; //Affects gamepad1's controls, switches the function of the sticks
    boolean enoughToSnake = true; //See if turning radius doesn't extend to inside the robot
    boolean isSnakingLeft = false; //See if the snake drive is turning to the left

    public float leftPower = 0; // [cart variable TODO: remove if able]
    public float rightPower = 0;  // [cart variable TODO: remove if able]
    public int leftCnt = 0; // left motor target counter [cart variable]
    public int rightCnt = 0; // right motor target counter  [cart variable]

    final static int ONE_ROTATION = 538; // for new AndyMark-20 motor encoder one rotation
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.69; // inches per chassis motor rotation based on 1:1 gear ratio

    final static double IMU_ROTATION_RATIO_L = 0.8; // 0.84; // Ratio of IMU Sensor Left turn to prevent overshooting the turn.
    final static double IMU_ROTATION_RATIO_R = 0.8; // 0.84; // Ratio of IMU Sensor Right turn to prevent overshooting the turn.

    final static double INIT_DRIVE_RATIO_FL = 0.998; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_FR = 0.978; //control veering by lowering right motor power
    final static double INIT_DRIVE_RATIO_BL = 1.0; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_BR = 0.982; //control veering by lowering right motor power

    final static double WIDTH_BETWEEN_WHEELS = 12;
    final static double LENGTH_BETWEEN_WHEELS = 12;
    final static double NB_WIDTH_BETWEEN_WHEELS = 12;
    final static double NB_LENGTH_BETWEEN_WHEELS = 11;
    final static double DISTANCE_TO_CENTER_OF_GLYPH = 5.9;
    final static double NB_DISTANCE_FOR_ORBIT = 6.5;
    final static double MIN_TURNING_RADIUS = 13;
    final static double MAX_TURNING_RADIUS = 100;
    final static double THETA_FRONT = (Math.atan(DISTANCE_TO_CENTER_OF_GLYPH / (0.5 * WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double THETA_BACK = (Math.atan((DISTANCE_TO_CENTER_OF_GLYPH + LENGTH_BETWEEN_WHEELS) / (0.5 * WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double NB_THETA_FRONT = (Math.atan(NB_DISTANCE_FOR_ORBIT / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double NB_THETA_BACK = (Math.atan((NB_DISTANCE_FOR_ORBIT + NB_LENGTH_BETWEEN_WHEELS) / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    static double SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*LENGTH_BETWEEN_WHEELS)/(0.5*WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));
    static double NB_SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*NB_LENGTH_BETWEEN_WHEELS)/(0.5*NB_WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));

    public double motorPowerLeft;
    public double motorPowerRight;
    double motorPowerTurn;
    public double drivePowerRatio = 0.5; //Controls the upper cap on drive speed
    float drivePower = 0; //Controls the throttling of the drive

    public double servoPosFL;
    public double servoPosFR;
    public double servoPosBL;
    public double servoPosBR;

    double leftServoAngle;
    double rightServoAngle;
    double r_Value;
    double thetaOneCalc;
    double thetaTwoCalc;
    double insideWheelsMod;
    double outsideWheelsMod;

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    final static double CRAB_DIFF_INC = 0.4663;
    final static double CRAB_DIFF_DEC = 0.4762;
    public final static double LEFT_SV_DIFF = 0.004;
    public final static double RIGHT_SV_DIFF = 0.004;
    final static double SERVO_FL_FORWARD_POSITION = 0.5;
    final static double SERVO_FR_FORWARD_POSITION = 0.4533;
    final static double SERVO_BL_FORWARD_POSITION = 0.48;
    final static double SERVO_BR_FORWARD_POSITION = 0.49;

    /* variables for newbot */
    public static double NB_CRAB_DIFF_INC_FL = 0.4373;
    public static double NB_CRAB_DIFF_DEC_FR = 0.4612;
    public static double NB_CRAB_DIFF_INC_BL = 0.4413;
    public static double NB_CRAB_DIFF_DEC_BR = 0.4872;
    public static double NB_LEFT_SV_DIFF = 0.001;
    public static double NB_RIGHT_SV_DIFF = 0.001;

    final static double NB_SERVO_FL_FORWARD_POSITION = 0.5278;
    final static double NB_SERVO_FR_FORWARD_POSITION = 0.4478;
    final static double NB_SERVO_BL_FORWARD_POSITION = 0.4178;
    final static double NB_SERVO_BR_FORWARD_POSITION = 0.5589;

    static double SERVO_FL_STRAFE_POSITION = SERVO_FL_FORWARD_POSITION + CRAB_DIFF_INC - LEFT_SV_DIFF;
    static double SERVO_FR_STRAFE_POSITION = SERVO_FR_FORWARD_POSITION - CRAB_DIFF_DEC + RIGHT_SV_DIFF;
    static double SERVO_BL_STRAFE_POSITION = SERVO_BL_FORWARD_POSITION + CRAB_DIFF_INC - LEFT_SV_DIFF;
    static double SERVO_BR_STRAFE_POSITION = SERVO_BR_FORWARD_POSITION - CRAB_DIFF_DEC + RIGHT_SV_DIFF;

    static double SERVO_FL_TURN_POSITION = SERVO_FL_FORWARD_POSITION - (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_FR_TURN_POSITION = SERVO_FR_FORWARD_POSITION + (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BL_TURN_POSITION = SERVO_BL_FORWARD_POSITION + (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BR_TURN_POSITION = SERVO_BR_FORWARD_POSITION - (SPOT_TURN_ANGLE_OFFSET);

    static double SERVO_FL_ORBIT_POSITION = SERVO_FL_FORWARD_POSITION + (THETA_FRONT / 180);
    static double SERVO_FR_ORBIT_POSITION = SERVO_FR_FORWARD_POSITION - (THETA_FRONT / 180);
    static double SERVO_BL_ORBIT_POSITION = SERVO_BL_FORWARD_POSITION + (THETA_BACK / 180);
    static double SERVO_BR_ORBIT_POSITION = SERVO_BR_FORWARD_POSITION - (THETA_BACK / 180);

    ElapsedTime runtime;
    public double target_heading = 0.0; // [cart variable]

    public DigitalChannel proxL = null;
    public DigitalChannel proxR = null;
    public DigitalChannel proxFL = null;
    public DigitalChannel proxML = null;
    public ModernRoboticsI2cRangeSensor rangeSensorFrontRight = null;
    public ModernRoboticsI2cRangeSensor rangeSensorFrontLeft = null;
    public ModernRoboticsI2cRangeSensor rangeSensorBack = null;
    // The IMU sensor object
    public BNO055IMU imu = null;
    public BNO055IMU imu2 = null;
    Acceleration accel = null;

    // State used for updating telemetry
    Orientation angles; // [ convert to local variable]
    Acceleration gravity;
    public enum RangeSensor{
        FRONT_LEFT, FRONT_RIGHT, BACK
    }
    public enum CarMode {
        CAR,
        STRAIGHT,
        CRAB,
        TURN,
        ORBIT
    }
    public CarMode cur_mode = CarMode.CAR;
    public CarMode old_mode = CarMode.CAR;
    ElapsedTime gTime;

    // Central core of robot
    CoreSystem core;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    SwerveSystem(CoreSystem core) {
        this.core = core;
    }

    boolean opModeIsActive() { // worksround for now until find a way to get real opModeIsActive
        return (gTime.seconds()<30.0);
    }
    public void enable(boolean isAuto) {
        use_swerve = true;   // use four motors and four servos for chassis
        use_front_drive_only = false;
        use_encoder = true;
        if (isAuto) {
            use_imu = true;
            use_imu2 = false;
            use_range_sensor = true;
            use_proximity_sensor = true;
        } else {
            use_imu = false;
            use_imu2 = false;
            use_range_sensor = false;
            use_proximity_sensor = false;
        }
    }

    void disable () {
        use_swerve = false;   // disable four motors and four servos for chassis
        use_front_drive_only = false;
        use_encoder = false;
        use_imu = false;
        use_imu2 = false;
        use_range_sensor = false;
        use_proximity_sensor = false;
    }

    void init(HardwareMap hwMap) {
        if (use_imu) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            imuParameters.loggingEnabled = true;
            imuParameters.loggingTag = "IMU";
            imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(imuParameters);

            // imu2 = hwMap.get(BNO055IMU.class, "imu2");
            // imu2.initialize(imuParameters);

            accel = imu.getAcceleration();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }
        if (use_verbose) {
            core.telemetry.addData("0: initialize imu CPU time =", "%3.2f sec", core.run_seconds());
            core.telemetry.update();
        }

        if (use_proximity_sensor) {
            proxL = hwMap.get(DigitalChannel.class, "proxL");
            proxL.setMode(DigitalChannel.Mode.INPUT);
            proxR = hwMap.get(DigitalChannel.class, "proxR");
            proxR.setMode(DigitalChannel.Mode.INPUT);
            proxFL = hwMap.get(DigitalChannel.class, "proxFL");
            proxFL.setMode(DigitalChannel.Mode.INPUT);
            proxML= hwMap.get(DigitalChannel.class, "proxML");
            proxML.setMode(DigitalChannel.Mode.INPUT);
        }
        if (use_range_sensor) {
            // rangeSensorFrontRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontRight");
            rangeSensorFrontLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontLeft");
            rangeSensorBack = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsBack");
        }
        if (use_verbose) {
            core.telemetry.addData("0: initialize prox/ranger sensors CPU time =", "%3.2f sec", core.run_seconds());
            core.telemetry.update();
        }

        if (use_swerve) {
            // Define and Initialize Motors
            motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
            motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
            if (!use_front_drive_only) {
                motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
                motorBackRight = hwMap.dcMotor.get("motorBackRight");
            }
            servoFrontRight = hwMap.servo.get("servoFrontRight");
            servoFrontLeft = hwMap.servo.get("servoFrontLeft");
            servoBackLeft = hwMap.servo.get("servoBackLeft");
            servoBackRight = hwMap.servo.get("servoBackRight");

            motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            if (!use_front_drive_only) {
                motorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                motorBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            }
            // Set all motors to zero power and set all servos to central position
            // May want to change servo #'s to the value where all wheels are pointing forward.
            set_chassis_forward_position();
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            if (!use_front_drive_only) {
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }

            // Set all motors to run with encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (!use_front_drive_only) {
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (!use_front_drive_only) {
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (use_swerve) {
                initialize_newbot();
            }
            if (use_verbose) {
                core.telemetry.addData("0: initialize chassis CPU time =", "%3.2f sec", core.run_seconds());
                core.telemetry.update();
            }
        }
    }

    void set_chassis_forward_position() {
        {
            servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(NB_SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(NB_SERVO_BR_FORWARD_POSITION);

            servoPosFL = NB_SERVO_FL_FORWARD_POSITION;
            servoPosFR = NB_SERVO_FR_FORWARD_POSITION;
            servoPosBL = NB_SERVO_BL_FORWARD_POSITION;
            servoPosBR = NB_SERVO_BR_FORWARD_POSITION;
        }
    }

    public void initialize_newbot() {
        SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*NB_LENGTH_BETWEEN_WHEELS)/(0.5*NB_WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));

        SERVO_FL_STRAFE_POSITION = NB_SERVO_FL_FORWARD_POSITION + NB_CRAB_DIFF_INC_FL - NB_LEFT_SV_DIFF;
        if (SERVO_FL_STRAFE_POSITION>1.0)
            SERVO_FL_STRAFE_POSITION = 1.0;
        SERVO_FR_STRAFE_POSITION = NB_SERVO_FR_FORWARD_POSITION - NB_CRAB_DIFF_DEC_FR + NB_RIGHT_SV_DIFF;
        if (SERVO_FR_STRAFE_POSITION<0.0)
            SERVO_FR_STRAFE_POSITION = 0.0;
        SERVO_BL_STRAFE_POSITION = NB_SERVO_BL_FORWARD_POSITION + NB_CRAB_DIFF_INC_BL - NB_LEFT_SV_DIFF;
        if (SERVO_BL_STRAFE_POSITION>1.0)
            SERVO_BL_STRAFE_POSITION = 1.0;
        SERVO_BR_STRAFE_POSITION = NB_SERVO_BR_FORWARD_POSITION - NB_CRAB_DIFF_DEC_BR + NB_RIGHT_SV_DIFF;
        if (SERVO_BR_STRAFE_POSITION<0.0)
            SERVO_BR_STRAFE_POSITION = 0.0;
        double NB_SERVO_UNIT_CONVERSION = Math.abs(SERVO_FL_STRAFE_POSITION - SERVO_FL_FORWARD_POSITION)/0.5;

        SERVO_FL_TURN_POSITION = NB_SERVO_FL_FORWARD_POSITION  - (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_FR_TURN_POSITION = NB_SERVO_FR_FORWARD_POSITION  + (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_BL_TURN_POSITION = NB_SERVO_BL_FORWARD_POSITION  + (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_BR_TURN_POSITION = NB_SERVO_BR_FORWARD_POSITION  - (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);

        SERVO_FL_ORBIT_POSITION = NB_SERVO_FL_FORWARD_POSITION + (NB_THETA_FRONT / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_FR_ORBIT_POSITION = NB_SERVO_FR_FORWARD_POSITION - (NB_THETA_FRONT / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_BL_ORBIT_POSITION = NB_SERVO_BL_FORWARD_POSITION + (NB_THETA_BACK / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_BR_ORBIT_POSITION = NB_SERVO_BR_FORWARD_POSITION - (NB_THETA_BACK / 180 * NB_SERVO_UNIT_CONVERSION);
    }

    public double imu_heading() {
        if (!use_imu)
            return 999;

        {
            if(!use_imu2){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return angles.firstAngle;
            }
            else{
                angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return angles.firstAngle;
            }
        }
    }

    public void reset_prox(){
        proxL.setMode(DigitalChannel.Mode.OUTPUT);
        proxR.setMode(DigitalChannel.Mode.OUTPUT);
        proxL.setState(true);
        proxR.setState(true);
        proxL.setMode(DigitalChannel.Mode.INPUT);
        proxR.setMode(DigitalChannel.Mode.INPUT);

        proxFL.setMode(DigitalChannel.Mode.OUTPUT);
        proxML.setMode(DigitalChannel.Mode.OUTPUT);
        proxFL.setState(true);
        proxML.setState(true);
        proxFL.setMode(DigitalChannel.Mode.INPUT);
        proxML.setMode(DigitalChannel.Mode.INPUT);
        core.sleep(200);
    }

    public double getRange(RangeSensor direction){
        ElapsedTime elapsedTime = new ElapsedTime();
        double distance = 999;
        if (!use_range_sensor)
            return 0.0;
        if(direction == RangeSensor.FRONT_LEFT){
            if (rangeSensorFrontLeft==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = rangeSensorFrontLeft.getDistance(DistanceUnit.CM);
            }
        } else if(direction == RangeSensor.FRONT_RIGHT){
            if (rangeSensorFrontRight==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = rangeSensorFrontRight.getDistance(DistanceUnit.CM);
            }
        } else if(direction == RangeSensor.BACK){
            if (rangeSensorBack==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = rangeSensorBack.getDistance(DistanceUnit.CM);
            }
        }
        else {
            throw new IllegalArgumentException("Direction not specified!");
        }
        if (distance>365) distance = 999;
        return distance;
    }

    public void driveTT(double lp, double rp) {
        if(!fast_mode && straight_mode) { // expect to go straight
            if (use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - target_heading) / 360);
                if(use_swerve) {
                    if(cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                                if (cur_heading - target_heading > 0.7) {
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - target_heading < -0.7) {
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                        }
                        else{ // When going backward
                            {
                                if (cur_heading - target_heading > 0.7) { //Drifting to the left
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - target_heading < -0.7) { //Drifting to the right
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            }
                        }
                    }
                    else if(cur_mode == CarMode.CRAB){  //Tentative, could stand to remove
                        if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                            if (rp > 0) rp *= 0.7; //If the robot is going forward
                            else lp *= 0.7; // If the robot is going backwards
                        } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                            if (lp > 0) lp *= 0.7;
                            else rp *= 0.7;
                        }
                    }
                }
            }
        }
        if (use_swerve) {
            if (cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
                motorFrontRight.setPower(rp);
                motorFrontLeft.setPower(lp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(lp);
                    motorBackRight.setPower(rp);
                }

            } else if(cur_mode == CarMode.CRAB) {
                motorFrontRight.setPower(lp);
                motorFrontLeft.setPower(-lp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(-rp);
                    motorBackRight.setPower(rp);
                }
            }
            else if(cur_mode == CarMode.TURN) {
                motorFrontRight.setPower(rp);
                motorFrontLeft.setPower(lp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(lp);
                    motorBackRight.setPower(rp);
                }
            }
        }
    }

    public void driveTTSnake(double drivePower, float turnIntensity, boolean snakeRight){ //Turn intensity is a value 0 to 1 meant to represent the triggers for determining the snake angle
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        calc_snake(snakeRight?0:turnIntensity,snakeRight?turnIntensity:0);
        snake_servo_adj();
        {
            insideWheelsMod = drivePower * ((Math.pow((Math.pow(0.5 * NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((r_Value) - NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (r_Value));
            outsideWheelsMod = drivePower * ((Math.pow((Math.pow(0.5 * NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((r_Value) + NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (r_Value));
        }
        if (!snakeRight) {
            motorPowerLeft = insideWheelsMod;
            motorPowerRight = outsideWheelsMod;
        } else {
            motorPowerLeft = outsideWheelsMod;
            motorPowerRight = insideWheelsMod;
        }
        motorFrontRight.setPower(motorPowerRight);
        motorFrontLeft.setPower(motorPowerLeft);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void driveTTCoast(double lp, double rp){
        boolean strafeRight = false;
        if(lp > 0 && rp > 0) {
            strafeRight = true;
        }
        else{
            strafeRight = false;
        }
        if(cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
            if (!use_front_drive_only) {
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        else if(cur_mode == CarMode.CRAB && strafeRight){
            if (!use_front_drive_only) {
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(cur_mode == CarMode.CRAB && !strafeRight){
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (!use_front_drive_only) {
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        if(!fast_mode && straight_mode) { // expect to go straight
            if (use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - target_heading) / 360);
                if(use_swerve) {
                    if(cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                            if (use_swerve) {
                                if (cur_heading - target_heading > 0.7) {
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - target_heading < -0.7) {
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            }
                        }
                        else{ // When going backward
                            if (use_swerve) {
                                if (cur_heading - target_heading > 0.7) { //Drifting to the left
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - target_heading < -0.7) { //Drifting to the right
                                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            }
                        }
                    }
                    else if(cur_mode == CarMode.CRAB){  //Tentative, could stand to remove
                        if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                            if (rp > 0) rp *= 0.7; //If the robot is going forward
                            else lp *= 0.7; // If the robot is going backwards
                        } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                            if (lp > 0) lp *= 0.7;
                            else rp *= 0.7;
                        }
                    }
                }
            }
        }
        if(use_swerve) {
            if (cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
                motorFrontRight.setPower(rp);
                motorFrontLeft.setPower(lp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                }

            } else if(cur_mode == CarMode.CRAB && strafeRight) {
                motorFrontRight.setPower(rp);
                motorFrontLeft.setPower(0);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(lp);
                }
            }
            else if(cur_mode == CarMode.CRAB && !strafeRight) {
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(rp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(lp);
                    motorBackRight.setPower(0);
                }
            }
            else if(cur_mode == CarMode.TURN) {
                motorFrontRight.setPower(rp);
                motorFrontLeft.setPower(lp);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(lp);
                    motorBackRight.setPower(rp);
                }
            }
        }
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        runtime.reset();
        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        int leftTC0 = 0;
        int rightTC0 = 0;
        int targetPosFrontLeft;
        int curPosFrontLeft = motorFrontLeft.getCurrentPosition();
        int targetPosFrontRight;
        int curPosFrontRight = motorFrontRight.getCurrentPosition();
        int targetPosBackLeft;
        int curPosBackLeft = (motorBackLeft!=null?motorBackLeft.getCurrentPosition():0);
        int targetPosBackRight;
        int curPosBackRight = (motorBackRight!=null?motorBackRight.getCurrentPosition():0);
        double initLeftPower = leftPower;
        double initRightPower = rightPower;
        double leftPowerSign = leftPower/Math.abs(leftPower);
        double rightPowerSign = rightPower/Math.abs(rightPower);
        boolean strafeRight = false;

        if(leftPower > 0 && rightPower > 0){
            strafeRight = true;
        }
        else{
            strafeRight = false;
        }

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!use_front_drive_only) {
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Math.abs(leftPower) > 0.4 && leftTC1 > 600 && !fast_mode) {
            leftTC2 = 150;
            leftTC0 = 75;
            leftTC1 -= 225;
        }
        if (Math.abs(rightPower) > 0.4 && rightTC1 > 600 && !fast_mode) {
            rightTC2 = 150;
            rightTC0 = 75;
            rightTC1 -= 225;
        }
        if(cur_mode == CarMode.STRAIGHT || cur_mode == CarMode.CAR) {
            if (rightTC0 > 0 || leftTC0 > 0) {
                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC0);
                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorFrontRight.setTargetPosition(targetPosFrontRight);

                runtime.reset();
                driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && (runtime.seconds() < 1) && opModeIsActive()) {
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    // show_telemetry();
                }
            }
            curPosFrontLeft = motorFrontLeft.getCurrentPosition();
            curPosFrontRight = motorFrontRight.getCurrentPosition();

            targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
            targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC1);

            motorFrontLeft.setTargetPosition(targetPosFrontLeft);
            motorFrontRight.setTargetPosition(targetPosFrontRight);

            driveTTCoast(leftPower, rightPower);
            int iter = 0;
            int prev_lpos = curPosFrontLeft;
            double cur_time = runtime.nanoseconds()/1000000000.0;
            double prev_time = cur_time;
            double cur_speed = 0, prev_speed=0;
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && (runtime.seconds() < 7) && opModeIsActive()) {
                driveTTCoast(leftPower, rightPower);
                if ((++iter)%6==0 && stop_on_bump==true) {
                    curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                    cur_time = runtime.nanoseconds()/1000000000.0;
                    cur_speed = (curPosFrontLeft-prev_lpos) / (cur_time-prev_time);
                    if (iter>12 && Math.abs(cur_speed-prev_speed)<1.0 &&
                            Math.abs(cur_speed)<0.25 && Math.abs(curPosFrontLeft-targetPosFrontLeft)>40) {
                        bump_detected = true;
                        break;
                    }
                    prev_lpos = curPosFrontLeft;
                    prev_time = cur_time;
                    prev_speed = cur_speed;
                }
                if (use_verbose) {
                    core.telemetry.addData("4.Speed cur/prev/i=", "%.2f/%.2f/%1d", cur_speed, prev_speed, iter);
                    core.telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                    core.telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                    core.telemetry.update();
                }
            }

            if (rightTC2 > 0 || leftTC2 > 0) {
                curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                curPosFrontRight = motorFrontRight.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC2);

                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorFrontRight.setTargetPosition(targetPosFrontRight);
                driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                while (!bump_detected && motorFrontLeft.isBusy() && motorFrontRight.isBusy() && (runtime.seconds() < 8) && opModeIsActive()) {
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                }
            }
            if (use_verbose) {
                //stop_chassis();
                core.telemetry.addData("4.Speed cur/prev/i/bumped=", "%.2f/%.2f/%1d/%s",
                        cur_speed, prev_speed, iter, (bump_detected?"T":"F"));
                core.telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                core.telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                core.telemetry.addLine("7.Hit B/X button to go next/exit ...");
                core.telemetry.update();
                //while (!gamepad1.x&&!gamepad1.b) {;}
            }
        }
        else if((cur_mode == CarMode.CRAB) && !use_front_drive_only){
            if(strafeRight) {
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC0);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC0);
                    motorFrontRight.setTargetPosition(targetPosFrontRight);
                    motorBackRight.setTargetPosition(targetPosBackRight);

                    runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (motorFrontRight.isBusy() && motorBackRight.isBusy() && (runtime.seconds() < 1) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        // show_telemetry();
                    }
                }
                curPosFrontRight = motorFrontRight.getCurrentPosition();
                curPosBackRight = motorBackRight.getCurrentPosition();

                targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC1);
                targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC1);

                motorFrontRight.setTargetPosition(targetPosFrontRight);
                motorBackRight.setTargetPosition(targetPosBackRight);

                driveTTCoast(leftPower, rightPower);
                while (motorFrontRight.isBusy() && motorBackRight.isBusy() && (runtime.seconds() < 7) && opModeIsActive()) {
                    driveTTCoast(leftPower, rightPower);
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontRight = motorFrontRight.getCurrentPosition();
                    curPosBackRight = motorBackRight.getCurrentPosition();

                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC2);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC2);

                    motorFrontRight.setTargetPosition(targetPosFrontRight);
                    motorBackRight.setTargetPosition(targetPosBackRight);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (motorFrontRight.isBusy() && motorBackRight.isBusy() && (runtime.seconds() < 8) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    }
                }
            }
            else{
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC0);
                    motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    motorBackLeft.setTargetPosition(targetPosBackLeft);

                    runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && (runtime.seconds() < 1) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        // show_telemetry();
                    }
                }
                curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                curPosBackLeft = motorBackLeft.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
                targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC1);

                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorBackLeft.setTargetPosition(targetPosBackLeft);

                driveTTCoast(leftPower, rightPower);
                while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && (runtime.seconds() < 7) && opModeIsActive()) {
                    driveTTCoast(leftPower, rightPower);
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                    curPosBackLeft = motorBackLeft.getCurrentPosition();

                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC2);

                    motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    motorBackLeft.setTargetPosition(targetPosBackLeft);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && (runtime.seconds() < 8) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    }
                }
            }
        }
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!use_front_drive_only) {
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!use_front_drive_only) {
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        stop_chassis();
        runtime.reset();
    }

    void StraightR(double power, double n_rotations) throws InterruptedException {
        straight_mode = true;
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        leftPower = rightPower = (float) power;
        run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
        straight_mode = false;
        if(cur_mode == CarMode.CRAB) {
            servoFrontLeft.setPosition(SERVO_FL_STRAFE_POSITION);
            servoFrontRight.setPosition(SERVO_FR_STRAFE_POSITION);
            servoBackLeft.setPosition(SERVO_BL_STRAFE_POSITION);
            servoBackRight.setPosition(SERVO_BR_STRAFE_POSITION);
        } else {
            set_chassis_forward_position();
        }

        //if (!fast_mode)
        //    sleep(135);
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        if (use_imu) {
            target_heading = imu_heading();
        }
        if (use_encoder) {
            double numberR = in / INCHES_PER_ROTATION;
            if(cur_mode == CarMode.CRAB){
                StraightR(power, numberR);
            }
            else {
                StraightR(-power, numberR);
            }
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            core.sleep(msec);
            driveTT(0, 0);
        }
    }

    public void StraightCm(double power, double cm) throws InterruptedException {
        if (use_imu) {
            target_heading = imu_heading();
        }
        if (use_encoder) {
            double WHEEL_DIAMETER = 102.18; // In millimeters
            double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // Still in millimeters
            double CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // Also is mm per rotation

            double numberR = cm / (CIRCUMFERENCE / 10);
            if(cur_mode == CarMode.CRAB){
                StraightR(power, numberR);
            }
            else {
                StraightR(-power, numberR);
            }
        } else { // using timer
            double cm_per_ms = 0.014 * power / 0.8;
            if (cm_per_ms < 0) cm_per_ms *= -1.0;
            long msec = (long) (cm / cm_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            core.sleep(msec);
            driveTT(0, 0);

        }
    }

    public void StraightTime(double power, double timeSec){
        driveTT(-power, -power);
        core.sleep((long)(timeSec * 1000));
        driveTT(0,0);
    }

    public void TurnRightD(double power, double degree) throws InterruptedException {

        double adjust_degree_imu = IMU_ROTATION_RATIO_R * degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        runtime.reset();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);

        leftPower = (float) -power;
        rightPower = (float) power;

        change_swerve_pos(CarMode.TURN);

        core.sleep(100);

        leftCnt += leftEncode;
        rightCnt += rightEncode;

        //DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos - adjust_degree_imu;
            if (target_heading <= -180) {
                target_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= target_heading) && (runtime.seconds() < 4.0) && opModeIsActive()) {
                current_pos = imu_heading();
                // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                core.sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        if (!opModeIsActive()) return;
        core.sleep(100);
        change_swerve_pos(old_mode);
        //if (!fast_mode)
        //    core.sleep(135);
    }

    public void TurnLeftD(double power, double degree) throws InterruptedException {
        double adjust_degree_imu = IMU_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        runtime.reset();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);

        leftPower = (float) power;
        rightPower = (float) -power;

        change_swerve_pos(CarMode.TURN);

        core.sleep(100);

        leftCnt += leftEncode;
        rightCnt += rightEncode;


        //DbgLog.msg(String.format("imu Left Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos + adjust_degree_imu;
            if (target_heading >= 180) {
                target_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            //DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
            while ((current_pos <= target_heading) && (runtime.seconds() < 5.0) && opModeIsActive()) {
                current_pos = imu_heading();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                core.sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        core.sleep(100);
        change_swerve_pos(old_mode);
        //if (!fast_mode)
        //    core.sleep(135);
    }

    // [Invoked from TaintedAccess]
    public void stop_chassis() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        if (!use_front_drive_only) {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
    }

    void reset_chassis()  {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (!use_front_drive_only) {
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftCnt = 0;
        rightCnt = 0;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!use_front_drive_only) {
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void alignUsingIMU(double power, double tar_degree) throws InterruptedException {
        // target degree must between -180 and +180
        // align up to 45 degree
        double offset = 0;
        double cur_adj_heading = imu_heading();
        if (tar_degree>135 && cur_adj_heading<0) {
            cur_adj_heading += 360;
        } else if (tar_degree<-135 && cur_adj_heading>0) {
            cur_adj_heading -= 360;
        }

        double imu_diff = cur_adj_heading-tar_degree;
        double corrected_degree = Math.abs(imu_diff)-0.5;
        if (corrected_degree>10) {
            corrected_degree*=0.8;
        }
        if (corrected_degree>45)
            corrected_degree = 45;
        if (imu_diff < -2) {
            TurnLeftD(power, corrected_degree);
        }
        else if (imu_diff > 2) {
            TurnRightD(power, corrected_degree);
        }
        change_swerve_pos(CarMode.CAR);
    }

    public void snake_servo_adj(){
        if(enoughToSnake) {
            if(isSnakingLeft) {
                leftServoAngle = 1 - (thetaOneCalc);
                rightServoAngle = 1 - (thetaTwoCalc);
            }
            else{
                leftServoAngle = thetaTwoCalc;
                rightServoAngle = thetaOneCalc;
            }
            servoPosFL = 1 - (leftServoAngle);
            servoPosFR = 1 - (rightServoAngle);
            servoPosBL = leftServoAngle;
            servoPosBR = rightServoAngle;
            servoFrontLeft.setPosition(servoPosFL);
            servoFrontRight.setPosition(servoPosFR);
            servoBackLeft.setPosition(servoPosBL);
            servoBackRight.setPosition(servoPosBR);
        }
        else {
            set_chassis_forward_position();
        }
    }

    public void car_servo_adj(double joy_stick_x){
        double degree = joy_stick_x * 25.0 / 180.0; // maximum 25 degree each way
        if(Math.abs(degree)>0.01) {

            servoPosFL = SERVO_FL_FORWARD_POSITION - degree;
            servoPosFR = SERVO_FR_FORWARD_POSITION - degree;
            servoFrontLeft.setPosition(servoPosFL);
            servoFrontRight.setPosition(servoPosFR);
        }
        else {
            set_chassis_forward_position();
        }
    }

    public void correct_swerve_servos(){
        // Normalize the values so neither exceed 1.0 or 0.0

        if (Math.abs(motorPowerLeft) > 1) {
            motorPowerLeft = 1.0;
        }
        if (Math.abs(motorPowerRight) > 1) {
            motorPowerRight = 1.0;
        }
        if (Math.abs(servoPosFL) > 1) {
            servoPosFL = 1.0;
        }
        if (Math.abs(servoPosFR) > 1) {
            servoPosFR = 1.0;
        }
        if (Math.abs(servoPosBL) > 1) {
            servoPosBL = 1.0;
        }
        if (Math.abs(servoPosBR) > 1) {
            servoPosBR = 1.0;
        }
        if (Math.abs(servoPosFL) < 0) {
            servoPosFL = 0.0;
        }
        if (Math.abs(servoPosFR) < 0) {
            servoPosFR = 0.0;
        }
        if (Math.abs(servoPosBL) < 0) {
            servoPosBL = 0.0;
        }
        if (Math.abs(servoPosBR) < 0) {
            servoPosBR = 0.0;
        }
    }

    public void change_swerve_pos(CarMode new_mode) {
        if (new_mode == CarMode.TURN) {
            servoFrontLeft.setPosition(SERVO_FL_TURN_POSITION);
            servoFrontRight.setPosition(SERVO_FR_TURN_POSITION);
            servoBackLeft.setPosition(SERVO_BL_TURN_POSITION);
            servoBackRight.setPosition(SERVO_BR_TURN_POSITION);

            servoPosFL = SERVO_FL_TURN_POSITION;
            servoPosFR = SERVO_FR_TURN_POSITION;
            servoPosBL = SERVO_BL_TURN_POSITION;
            servoPosBR = SERVO_BR_TURN_POSITION;
        }
        else if(new_mode == CarMode.CRAB){
            servoFrontLeft.setPosition(SERVO_FL_STRAFE_POSITION);
            servoFrontRight.setPosition(SERVO_FR_STRAFE_POSITION);
            servoBackLeft.setPosition(SERVO_BL_STRAFE_POSITION);
            servoBackRight.setPosition(SERVO_BR_STRAFE_POSITION);

            servoPosFL = SERVO_FL_STRAFE_POSITION;
            servoPosFR = SERVO_FR_STRAFE_POSITION;
            servoPosBL = SERVO_BL_STRAFE_POSITION;
            servoPosBR = SERVO_BR_STRAFE_POSITION;
        }
        else if(new_mode == CarMode.STRAIGHT || new_mode == CarMode.CAR){
            set_chassis_forward_position();
        }
        else if(new_mode == CarMode.ORBIT){
            servoFrontLeft.setPosition(SERVO_FL_ORBIT_POSITION);
            servoFrontRight.setPosition(SERVO_FR_ORBIT_POSITION);
            servoBackLeft.setPosition(SERVO_BL_ORBIT_POSITION);
            servoBackRight.setPosition(SERVO_BR_ORBIT_POSITION);

            servoPosFL = SERVO_FL_ORBIT_POSITION;
            servoPosFR = SERVO_FR_ORBIT_POSITION;
            servoPosBL = SERVO_BL_ORBIT_POSITION;
            servoPosBR = SERVO_BR_ORBIT_POSITION;
        }
        old_mode = cur_mode;
        cur_mode = new_mode;

//        isTestingFL = false;
//        isTestingFR = false;
//        isTestingBL = false;
//        isTestingBR = false;
    }

    public void set_swerve_power(float right_stick /*turning power*/, float left_stick /*drive power*/,
                                 float left_t, float right_t, boolean isK){
        float x_stick = 0;
        if (left_t > 0.1)
            x_stick = -1 * left_t;
        else x_stick = right_t;

        if(cur_mode == CarMode.CAR) {

            insideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((r_Value) - NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (r_Value));
            outsideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((r_Value) + NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (r_Value));


            if (enoughToSnake) {
                if (!use_front_drive_only) {
                    motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                if (isSnakingLeft) {
                    motorPowerLeft = insideWheelsMod;
                    motorPowerRight = outsideWheelsMod;
                } else {
                    motorPowerLeft = outsideWheelsMod;
                    motorPowerRight = insideWheelsMod;
                }
            } else {
                if (!use_front_drive_only) {
                    motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                motorPowerLeft = left_stick;
                motorPowerRight = left_stick;
            }

            if(isK) {

                if (enoughToSnake) {
                    motorFrontLeft.setPower(motorPowerLeft * drivePowerRatio);
                    motorFrontRight.setPower(motorPowerRight * drivePowerRatio);
                    if (!use_front_drive_only) {
                        motorBackLeft.setPower(0);
                        motorBackRight.setPower(0);
                    }
                } else {
                    motorFrontLeft.setPower(motorPowerLeft * drivePowerRatio);
                    motorFrontRight.setPower(motorPowerRight * drivePowerRatio);
                    if (!use_front_drive_only) {
                        motorBackLeft.setPower(motorPowerLeft * drivePowerRatio);
                        motorBackRight.setPower(motorPowerRight * drivePowerRatio);
                    }
                }
            }
            else{
                if (enoughToSnake) {
                    motorFrontLeft.setPower(motorPowerLeft);
                    motorFrontRight.setPower(motorPowerRight);
                    if (!use_front_drive_only) {
                        motorBackLeft.setPower(motorPowerLeft);
                        motorBackRight.setPower(motorPowerRight);
                    }
                    else{
                        motorBackLeft.setPower(0);
                        motorBackRight.setPower(0);
                    }
                } else {
                    motorFrontLeft.setPower(motorPowerLeft);
                    motorFrontRight.setPower(motorPowerRight);
                    if (!use_front_drive_only) {
                        motorBackLeft.setPower(motorPowerLeft);
                        motorBackRight.setPower(motorPowerRight);
                    }
                }
            }

        } else if (cur_mode == CarMode.TURN) {
            motorPowerTurn = x_stick;
            motorFrontLeft.setPower(-motorPowerTurn * drivePowerRatio);
            motorFrontRight.setPower(motorPowerTurn * drivePowerRatio);
            if (!use_front_drive_only) {
                motorBackLeft.setPower(-motorPowerTurn * drivePowerRatio);
                motorBackRight.setPower(motorPowerTurn * drivePowerRatio);
            }
        } else {
            motorPowerLeft = left_stick;
            motorPowerRight = right_stick;
            if (cur_mode == CarMode.STRAIGHT) {
                motorFrontLeft.setPower(motorPowerLeft * drivePowerRatio);
                motorFrontRight.setPower(motorPowerRight * drivePowerRatio);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(motorPowerLeft * drivePowerRatio);
                    motorBackRight.setPower(motorPowerRight * drivePowerRatio);
                }
            } else if(cur_mode == CarMode.CRAB) {
                motorFrontLeft.setPower(-motorPowerRight * drivePowerRatio);
                motorFrontRight.setPower(motorPowerRight * drivePowerRatio);
                if (!use_front_drive_only) {
                    motorBackLeft.setPower(motorPowerLeft * drivePowerRatio);
                    motorBackRight.setPower(-motorPowerLeft * drivePowerRatio);
                }
            }
        }
    }

    // void calc_snake(float stick_x){
    public void calc_snake(float left_t, float right_t){
        float stick_x;
        if (left_t > 0.1)
            stick_x = -1 * left_t;
        else stick_x = right_t;
        if(stick_x > 0.1){
            isSnakingLeft = false;
        }
        else{
            isSnakingLeft = true;
        }

        if(Math.abs(stick_x) < 0.2){
            enoughToSnake = false;
        }
        else{
            enoughToSnake = true;
        }

        if(Math.abs(stick_x) > 0.8){
            r_Value = MAX_TURNING_RADIUS - (/*Hypothetical limiter here*/(MAX_TURNING_RADIUS - MIN_TURNING_RADIUS));
        }
        else{
            r_Value = MAX_TURNING_RADIUS - (Math.abs(stick_x) * (MAX_TURNING_RADIUS - MIN_TURNING_RADIUS));
        }

        {
            thetaOneCalc = (Math.atan((0.5 * NB_WIDTH_BETWEEN_WHEELS) / ((r_Value) - (0.5 * NB_LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 1 (inside wheels)
            thetaTwoCalc = (Math.atan((0.5 * NB_WIDTH_BETWEEN_WHEELS) / ((r_Value) + (0.5 * NB_LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 2 (outside wheels)
        }

    }

}
