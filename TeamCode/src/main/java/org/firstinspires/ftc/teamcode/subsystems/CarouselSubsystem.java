package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * CarouselSubsystem
 *
 * This subsystem is just for operating the carousel for delivering ducks
 *
 * It has three functions:
 * forward()    rotate forward
 * backward()   rotate backward
 * stop()       stop rotation
 *
 * The motor is a Servo set in continuous rotation mode
 *
 */

public class CarouselSubsystem extends SubsystemBase {
    private CRServo m_servo;
    private RobotContainer m_robot;

    private static final boolean DEBUG = false;

    /**
     *
     * @param hardwareMap   The robot hardware map
     * @param robot         The container object that holds all the robot subsystems
     */
    public CarouselSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        m_robot = robot;

        m_servo = new CRServo( hardwareMap, Constants.RobotConfigConstants.ROBOT_CAROUSEL_SERVO );
        m_servo.setRunMode(Motor.RunMode.RawPower);
        m_servo.stop();
    }

    /**
     * Set the servo motor power
     *
     * @param power A number from -1 to 1
     */
    public void setPower(double power)
    {
        m_servo.set(power);
    }

    public void forward()
    {
        if (DEBUG) Log.w("CAROUSEL", "set forward");
        setPower(1);
    }
    public void backward()
    {
        if (DEBUG) Log.w("CAROUSEL", "set backward");
        setPower(-1);
    }
    public void stop()
    {
        if (DEBUG) Log.w("CAROUSEL", "stop");
        setPower(0);
    }
}
