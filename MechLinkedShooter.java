/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MechLinkedShooter", group="Linear OpMode")
public class MechLinkedShooter extends LinearOpMode {

    // ---------------- Launcher tuning ----------------
    // Two launcher motors are mechanically linked, so they share ONE control loop
    // that runs here in the OpMode. Both motors receive the same power every loop.
    // Do NOT use setVelocity() on them - that would start two hub PID loops that
    // fight each other through the shared shaft.

    // Encoder ticks per output-shaft revolution of the launcher motors.
    //   goBILDA 5202/5203 1:1 (6000 RPM) bare motor = 28
    //   REV HD Hex bare motor                        = 28
    // If your launcher motors have a gearbox, multiply 28 by the gear ratio.
    static final double LAUNCHER_TICKS_PER_REV = 28.0;

    static final double LAUNCHER_DEFAULT_RPM = 3000.0;
    static final double LAUNCHER_RPM_STEP    = 100.0;   // D-pad up/down adjustment
    static final double LAUNCHER_MAX_RPM     = 6000.0;
    static final double LAUNCHER_READY_TOLERANCE_RPM = 100.0;

    // Set false if only ONE launcher motor has its encoder cable plugged in.
    // The plugged-in one must be "launcher_left" (or swap it in measureLauncherTps()).
    static final boolean USE_BOTH_ENCODERS = true;

    // Control gains (units: motor power per encoder tick/second).
    // Tuning: set KP = 0, adjust KV until actual RPM ~ target RPM at a few speeds,
    // then raise KP until the flywheel recovers quickly after a shot without oscillating.
    // Free speed of a 6000 RPM motor = 6000 * 28 / 60 = 2800 ticks/s, so KV starts at 1/2800.
    static final double LAUNCHER_KV = 1.0 / 2800.0;   // feedforward
    static final double LAUNCHER_KS = 0.0;            // static friction offset (power)
    static final double LAUNCHER_KP = 0.0005;         // proportional feedback

    // Gains above are tuned at this battery voltage; power is scaled to compensate.
    static final double NOMINAL_VOLTAGE = 12.0;
    // -------------------------------------------------

    // ---------------- Gate (feeder) tuning ----------------
    // Power sent to the gate CRServos while feeding (-1.0 to 1.0).
    // If the gates spin the wrong way, flip the directions in runOpMode().
    static final double GATE_FEED_POWER = 1.0;
    // ------------------------------------------------------

    // ---------------- Blocker gate (positional servo) ----------------
    // Servo positions are 0.0 to 1.0. These are starting guesses - tune them
    // on the robot so UP fully blocks the ball and DOWN fully clears it.
    static final double BLOCKER_UP_POSITION   = 0.0;
    static final double BLOCKER_DOWN_POSITION = 0.5;
    // -----------------------------------------------------------------

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private DcMotorEx launcherLeft = null;
    private DcMotorEx launcherRight = null;
    private VoltageSensor battery = null;
    private CRServo leftGate = null;
    private CRServo rightGate = null;
    private Servo blockerGate = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive     = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive    = hardwareMap.get(DcMotor.class, "right_drive");
        intake        = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcher_left");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher_right");
        leftGate      = hardwareMap.get(CRServo.class, "left_gate");
        rightGate     = hardwareMap.get(CRServo.class, "right_gate");
        blockerGate   = hardwareMap.get(Servo.class, "gate");
        battery       = hardwareMap.voltageSensor.iterator().next();

        // If the robot now drives BACKWARD when you push the left stick forward,
        // change BOTH of these to FORWARD.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // ---------- Launcher motors ----------
        // The two launcher motors usually face each other on the shared shaft, so one
        // is reversed. Set these so POSITIVE power spins both the shooting direction.
        // CHECK THIS BEFORE LINKING THEM MECHANICALLY - if they oppose each other
        // they will stall and overheat. Reversing a motor also flips its encoder,
        // so both should read positive velocity while shooting (check telemetry).
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{launcherLeft, launcherRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // RUN_WITHOUT_ENCODER: the hub applies our power directly (no hub PID),
            // but we can still read encoder velocity.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let flywheel coast
            m.setPower(0);
        }

        // The two gates usually face each other (mirrored), so one is reversed
        // so that the same positive power feeds balls toward the shooter on both.
        // Swap these if a gate pushes balls the wrong way.
        leftGate.setDirection(DcMotorSimple.Direction.FORWARD);
        rightGate.setDirection(DcMotorSimple.Direction.REVERSE);
        leftGate.setPower(0);
        rightGate.setPower(0);

        // Blocker starts UP so no ball can reach the shooter.
        // Note: a positional servo moves to this position as soon as INIT is pressed.
        blockerGate.setPosition(BLOCKER_UP_POSITION);

        double targetRpm = LAUNCHER_DEFAULT_RPM;
        boolean launcherOn = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ---------- Drive (POV) ----------
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // ---------- Intake ----------
            if (gamepad1.right_trigger > 0.2) {
                intake.setPower(1);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // ---------- Launcher on/off and speed ----------
            if (gamepad1.a) launcherOn = true;
            if (gamepad1.b) launcherOn = false;

            // D-pad up/down nudges target speed (one step per press)
            if (gamepad1.dpad_up && !prevDpadUp) {
                targetRpm = Math.min(targetRpm + LAUNCHER_RPM_STEP, LAUNCHER_MAX_RPM);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                targetRpm = Math.max(targetRpm - LAUNCHER_RPM_STEP, 0);
            }
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // ---------- Launcher control loop (single loop, two motors) ----------
            double targetTps   = rpmToTicksPerSec(targetRpm);
            double measuredTps = measureLauncherTps();
            double voltage     = battery.getVoltage();

            double launcherPower = 0.0;
            if (launcherOn) {
                double ff = LAUNCHER_KV * targetTps + LAUNCHER_KS;
                double fb = LAUNCHER_KP * (targetTps - measuredTps);
                double voltageScale = (voltage > 1.0) ? NOMINAL_VOLTAGE / voltage : 1.0;
                launcherPower = (ff + fb) * voltageScale;
                // Never drive the flywheel backwards to slow it; just let it coast.
                launcherPower = Range.clip(launcherPower, 0.0, 1.0);
            }
            launcherLeft.setPower(launcherPower);
            launcherRight.setPower(launcherPower);

            double actualRpm = ticksPerSecToRpm(measuredTps);
            boolean launcherReady = launcherOn
                    && Math.abs(actualRpm - targetRpm) < LAUNCHER_READY_TOLERANCE_RPM;

            // ---------- Gates (feed into shooter) ----------
            // Only feed when the driver is holding shoot AND the flywheel is at speed.
            // If the flywheel dips after a shot, the gates pause until it recovers,
            // which keeps every shot at the same speed.
            boolean shootPressed = gamepad1.right_bumper;
            boolean feeding = shootPressed && launcherReady;
            double gatePower = feeding ? GATE_FEED_POWER : 0.0;
            leftGate.setPower(gatePower);
            rightGate.setPower(gatePower);

            // Blocker is DOWN only while the feeders are running, UP otherwise.
            blockerGate.setPosition(feeding ? BLOCKER_DOWN_POSITION : BLOCKER_UP_POSITION);

            // ---------- Drive output ----------
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // ---------- Telemetry ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
            telemetry.addData("Launcher RPM", "target %.0f | actual %.0f", targetRpm, actualRpm);
            telemetry.addData("Launcher Enc RPM", "left %.0f | right %.0f",
                    ticksPerSecToRpm(launcherLeft.getVelocity()),
                    ticksPerSecToRpm(launcherRight.getVelocity()));
            telemetry.addData("Launcher Power", "%.2f  (battery %.1f V)", launcherPower, voltage);
            telemetry.addData("Launcher Ready", launcherReady ? "YES" : "no");
            telemetry.addData("Gates", feeding ? "FEEDING"
                    : (shootPressed ? "waiting for speed" : "stopped"));
            telemetry.addData("Blocker", feeding ? "DOWN" : "UP");
            telemetry.update();
        }

        // Make sure everything stops when the OpMode ends.
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        leftGate.setPower(0);
        rightGate.setPower(0);
    }

    /** Launcher speed in encoder ticks/second, from one or both encoders. */
    private double measureLauncherTps() {
        if (USE_BOTH_ENCODERS) {
            return (launcherLeft.getVelocity() + launcherRight.getVelocity()) / 2.0;
        }
        return launcherLeft.getVelocity();
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * LAUNCHER_TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / LAUNCHER_TICKS_PER_REV;
    }
}
