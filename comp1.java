package org.firstinspires.ftc.teamcode.tread;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Action;
import org.treadpathing.route.InstantAction;
import org.treadpathing.route.Route;

/**
 * A complete autonomous, showing every primitive the library has.
 *
 * <p>Read the route out loud and it describes what the robot does. That is the whole point of
 * the builder.
 *
 * <h3>Three things to notice</h3>
 *
 * <b>Headings are the robot's headings.</b> Every pose is where the nose points, which is what
 * you see standing at the field. On a reversed segment the library flips the travel direction
 * for you.
 *
 * <p><b>{@code reversed()} inserts a cusp.</b> The path being accumulated is closed off, the
 * robot comes to a stop, and a new path starts the other way. That is the only honest way for
 * a differential drive to change direction, and doing it explicitly beats meeting it as a
 * mystery lurch halfway through a match.
 *
 * <p><b>{@code stopAndHold} is what makes it accurate.</b> Drive segments get the robot
 * roughly where it is going, quickly. The pose hold is what gets it exactly there, and it is
 * where you should spend your route's time budget when position matters.
 */
@Autonomous(name = "Shoot/Park", group = "tread")
public class ShootPark extends LinearOpMode {

    static final double LAUNCHER_TICKS_PER_REV = 28.0;
    static final double LAUNCHER_TARGET_RPM = 2700.0;
    static final double LAUNCHER_READY_TOLERANCE_RPM = 100.0;
    static final boolean USE_CUSTOM_PIDF = false;
    static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 12.0);
    
    static final double GATE_FEED_POWER = 1.0;
    static final double BLOCKER_UP_POSITION   = 0.0;   // blocks the ball
    static final double BLOCKER_DOWN_POSITION = 0.5;   // lets the ball through
    
    static final double SPIN_UP_TIMEOUT = 3.0;   // give up waiting for speed after this (s)
    static final double SHOOT_TIME      = 10.0;   // how long to feed balls (s) - tune to empty the robot

    private DcMotorEx launcher;
    private CRServo leftGate;
    private CRServo rightGate;
    private Servo blockerGate;
    
    
    @Override
    public void runOpMode() {
        
          // ---------- Shooter hardware ----------
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftGate    = hardwareMap.get(CRServo.class, "left_gate");
        rightGate   = hardwareMap.get(CRServo.class, "right_gate");
        blockerGate = hardwareMap.get(Servo.class, "gate");

        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (USE_CUSTOM_PIDF) {
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        }

        leftGate.setDirection(DcMotorSimple.Direction.FORWARD);
        rightGate.setDirection(DcMotorSimple.Direction.REVERSE);
        setFeeders(0.0);
        blockerGate.setPosition(BLOCKER_UP_POSITION);
        
        Follower follower = Constants.buildFollower(hardwareMap);

        follower.setPose(new Pose(60.75, 9.0, Math.toRadians(-90.75)));
 
        // 1. Spin up and finish once the flywheel is within tolerance of target.
        Action spinUp = new Action() {
            @Override
            public void start() {
                launcher.setVelocity(rpmToTicksPerSec(LAUNCHER_TARGET_RPM));
            }

            @Override
            public boolean run() {
                return launcherReady();
            }
        };
        
        // 2. Open the gate and run the feeders for SHOOT_TIME seconds.
        //    Like the TeleOp, feeding pauses if the flywheel dips after a shot
        //    so every ball leaves at the same speed.
        Action shoot = new Action() {
            private double endTime;

            @Override
            public void start() {
                endTime = now() + SHOOT_TIME;
            }

            @Override
            public boolean run() {
                boolean feeding = launcherReady();
                blockerGate.setPosition(feeding ? BLOCKER_DOWN_POSITION : BLOCKER_UP_POSITION);
                setFeeders(feeding ? GATE_FEED_POWER : 0.0);
                return now() >= endTime;
            }
        };

        // 3. Close the gate and stop everything. A separate step so the shooter is
        //    always shut off, even if an earlier action hit its timeout.
        Action stopShooter = new InstantAction() {
            @Override
            public void execute() {
                setFeeders(0.0);
                blockerGate.setPosition(BLOCKER_UP_POSITION);
                launcher.setVelocity(0);
            }
        };


       Route route = follower.route()
        .action(spinUp, SPIN_UP_TIMEOUT)
        .action(shoot, SHOOT_TIME + 0.5)
        .action(stopShooter)
        .waitSeconds(0.50)
        .reversed()
        
        .reversed()
        .splineTo(new Pose(44.75, 34.25, Math.toRadians(-21.34)))
        .splineTo(new Pose(18.75, 51.50, Math.toRadians(-82.50)))
        .splineTo(new Pose(12.0, 89.0, Math.toRadians(-82.26)))
        .build();
        
        
        telemetry.addLine(route.summary());
        telemetry.addLine();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        follower.follow(route);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine(follower.telemetry());
            telemetry.update();
        }

        follower.breakFollowing();

        while (opModeIsActive()) {
            telemetry.addData("finished at", follower.getPose());
            telemetry.addData("planned", route.getPlannedEnd());
            telemetry.addData("error (in)", "%.2f",
                    follower.getPose().distanceTo(route.getPlannedEnd()));
            telemetry.update();
        }
    }
    
    private void setFeeders(double power) {
        leftGate.setPower(power);
        rightGate.setPower(power);
    }

    private boolean launcherReady() {
        return Math.abs(currentRpm() - LAUNCHER_TARGET_RPM) < LAUNCHER_READY_TOLERANCE_RPM;
    }

    private double currentRpm() {
        return launcher.getVelocity() * 60.0 / LAUNCHER_TICKS_PER_REV;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * LAUNCHER_TICKS_PER_REV / 60.0;
    }

    private static double now() {
        return System.nanoTime() * 1e-9;
    }
}
