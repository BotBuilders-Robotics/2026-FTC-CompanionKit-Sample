package org.firstinspires.ftc.teamcode.tread;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Action;
import org.treadpathing.route.InstantAction;
import org.treadpathing.route.Route;

/**
 * Simple Tread Pathing auto for the Companion Kit robot:
 *   1. Spin the launcher up to target RPM and wait until it is at speed.
 *   2. Open the blocker gate and run the gate (feeder) CRServos to shoot.
 *   3. Close the gate, stop the shooter, then follow the drive path.
 *
 * Hardware names and launcher/gate settings match CompKitDrive.java.
 * Lives in the tread package so it can use the quickstart Constants.buildFollower().
 * Plain Java 7 (anonymous classes, no lambdas) so it builds in OnBotJava.
 */
@Autonomous(name = "Tread Shoot Then Path", group = "tread")
public class TreadShootAuto extends LinearOpMode {

    // ---------------- Launcher (same as CompKitDrive) ----------------
    static final double LAUNCHER_TICKS_PER_REV = 28.0;
    static final double LAUNCHER_TARGET_RPM = 3000.0;
    static final double LAUNCHER_READY_TOLERANCE_RPM = 100.0;
    static final boolean USE_CUSTOM_PIDF = false;
    static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 12.0);

    // ---------------- Gates (same as CompKitDrive) ----------------
    static final double GATE_FEED_POWER = 1.0;
    static final double BLOCKER_UP_POSITION   = 0.0;   // blocks the ball
    static final double BLOCKER_DOWN_POSITION = 0.5;   // lets the ball through

    // ---------------- Auto timing ----------------
    static final double SPIN_UP_TIMEOUT = 3.0;   // give up waiting for speed after this (s)
    static final double SHOOT_TIME      = 3.0;   // how long to feed balls (s) - tune to empty the robot

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

        // ---------- Follower (drive uses left_drive / right_drive from Constants) ----------
        Follower follower = Constants.buildFollower(hardwareMap);
        Pose start = new Pose(9.0, 60.0, 0.0);   // <-- set to where the robot starts
        follower.setPose(start);

        // ---------- Actions ----------

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

        // ---------- Route ----------
        Route route = follower.route()
                .action(spinUp, SPIN_UP_TIMEOUT)
                .action(shoot, SHOOT_TIME + 0.5)
                .action(stopShooter)

                // ----- The path after shooting: replace with your own waypoints -----
                .splineTo(new Pose(34.0, 60.0, 0.0))
                .splineTo(new Pose(52.0, 84.0, MathUtil.toRadians(70.0)))
                .stopAndHold(1.5)
                .build();

        telemetry.addLine(route.summary());
        telemetry.addData("start", start);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        follower.follow(route);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Launcher RPM", "target %.0f | actual %.0f",
                    LAUNCHER_TARGET_RPM, currentRpm());
            telemetry.addData("Launcher Ready", launcherReady() ? "YES" : "no");
            telemetry.addLine(follower.telemetry());
            telemetry.update();
        }

        follower.breakFollowing();
        setFeeders(0.0);
        launcher.setVelocity(0);
    }

    // ---------- Helpers ----------

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
