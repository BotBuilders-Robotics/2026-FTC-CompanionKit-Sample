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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompKit", group="Linear OpMode")
public class CompKitDrive extends LinearOpMode {

    // ---------------- Launcher tuning ----------------
    // Encoder ticks per output-shaft revolution of the launcher motor.
    //   goBILDA 5202/5203 1:1 (6000 RPM) bare motor = 28
    //   REV HD Hex bare motor                        = 28
    // If your launcher motor has a gearbox, multiply 28 by the gear ratio.
    static final double LAUNCHER_TICKS_PER_REV = 28.0;

    // Starting target speed (RPM). Your old -0.55 power on a 6000 RPM motor
    // was roughly 3000 RPM, so start here and tune.
    static final double LAUNCHER_DEFAULT_RPM = 3000.0;
    static final double LAUNCHER_RPM_STEP    = 100.0;   // D-pad up/down adjustment
    static final double LAUNCHER_MAX_RPM     = 6000.0;
    static final double LAUNCHER_READY_TOLERANCE_RPM = 100.0;

    // Optional custom PIDF. Leave USE_CUSTOM_PIDF false to use the hub's defaults.
    static final boolean USE_CUSTOM_PIDF = false;
    static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 12.0);
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
    private DcMotorEx launcher = null;
    private CRServo leftGate = null;
    private CRServo rightGate = null;
    private Servo blockerGate = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        launcher   = hardwareMap.get(DcMotorEx.class, "launcher");
        leftGate   = hardwareMap.get(CRServo.class, "left_gate");
        rightGate  = hardwareMap.get(CRServo.class, "right_gate");
        blockerGate = hardwareMap.get(Servo.class, "gate");

        
        // If the robot now drives BACKWARD when you push the left stick forward,
        // change BOTH of these to FORWARD.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        
        // use positive velocities. (Reversing also flips the encoder, so the
        // measured velocity will read positive when shooting.)
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // enables velocity control
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let flywheel coast
        if (USE_CUSTOM_PIDF) {
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
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

            // ---------- Launcher (velocity control) ----------
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

            if (launcherOn) {
                launcher.setVelocity(rpmToTicksPerSec(targetRpm));
            } else {
                launcher.setVelocity(0);
            }

            double actualRpm = ticksPerSecToRpm(launcher.getVelocity());
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
            telemetry.addData("Launcher Ready", launcherReady ? "YES" : "no");
            telemetry.addData("Gates", feeding ? "FEEDING"
                    : (shootPressed ? "waiting for speed" : "stopped"));
            telemetry.addData("Blocker", feeding ? "DOWN" : "UP");
            telemetry.update();
        }
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * LAUNCHER_TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / LAUNCHER_TICKS_PER_REV;
    }
}
