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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Motion Eval", group="Idk What This Is For")
public class BasicMotionEval extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor[] motors;
    private DcMotor m0 = null;
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;


    // Servos
    private Servo[] servos;
    private Servo s0 = null;
    private Servo s1 = null;
    private Servo s2 = null;
    private Servo s3 = null;
    private Servo s4 = null;
    private Servo s5 = null;

    // Track power levels
    private double[] motorPowers = null;
    private double[] servoPowers = null;

    // DPad Action States
    private boolean[] lastDPadState = null;
    private boolean[] currentDPadState = null;

    private boolean dpadUpBtnPressed = false;
    private boolean dpadRightBtnPressed = false;
    private boolean dpadDownBtnPressed = false;
    private boolean dpadLeftBtnPressed = false;

    private boolean dpadUpBtnReleased = false;
    private boolean dpadRightBtnReleased = false;
    private boolean dpadDownBtnReleased = false;
    private boolean dpadLeftBtnReleased = false;

    // Servos
    private int[] activeServos = null;

    static final double SERVO_MAX_POS = 0.85;
    static final double SERVO_MIN_POS = 0.15;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        m0  = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2  = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        motors = new DcMotor[]{ m0, m1, m2, m3 };

        s0  = hardwareMap.get(Servo.class, "s0");
        s1  = hardwareMap.get(Servo.class, "s1");
        s2  = hardwareMap.get(Servo.class, "s2");
        s3  = hardwareMap.get(Servo.class, "s3");
        s4  = hardwareMap.get(Servo.class, "s4");
        s5  = hardwareMap.get(Servo.class, "s5");
        servos = new Servo[]{ s0, s1, s2, s3, s4, s5 };


        // Track power levels
        motorPowers = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        servoPowers = new double[]{ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

        // Init controllable servos
        activeServos = new int[]{ 0, 1 };

        // Init motors and servos
        for(int ndx=0; ndx < motors.length; ndx++) { motors[ndx].setPower(motorPowers[ndx]); }
        for(int ndx=0; ndx < servos.length; ndx++) { servos[ndx].setPosition(servoPowers[ndx]); }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Update DPad action (pressed/released) states
            // TODO: Only update this every XXXX Hz
            updateBtnActions();


            // Set motor powers
            motorPowers[0] = gamepad1.left_stick_x;
            motorPowers[1] = gamepad1.left_stick_y;
            motorPowers[2] = gamepad1.right_stick_x;
            motorPowers[3] = gamepad1.right_stick_y;
            for(int ndx=0; ndx < motors.length; ndx++) { motors[ndx].setPower(motorPowers[ndx]); }


            // Update active servos
            int sL = activeServos[0];
            int sR = activeServos[1];
            if (dpadRightBtnReleased) {
                sL++;
                if(sL == sR) { sL++; }
                if(sL >= servos.length) { sL=0; }
            }
            if (dpadLeftBtnReleased) {
                sL--;
                if(sL == sR) { sL--; }
                if(sL <= 0) { sL = servos.length-1; }
            }
            activeServos[0] = sL;
            activeServos[1] = sR;

            // Update active servo powers
            if(dpadUpBtnReleased) { servoPowers[sL] = SERVO_MAX_POS; }
            else if (dpadDownBtnReleased) { servoPowers[sL] = SERVO_MIN_POS; }
            else if (gamepad1.left_bumper) { servoPowers[sL] += 0.001; }
            else if (gamepad1.left_trigger >= 0.8) { servoPowers[sL] -= 0.001; }

            if (servoPowers[sL] > SERVO_MAX_POS) { servoPowers[sL] = SERVO_MAX_POS; }
            else if (servoPowers[sL] < SERVO_MIN_POS) {servoPowers[sL] = SERVO_MIN_POS; }

            // TODO: Add buttons for second servo


            // Set servo powers
            for(int ndx=0; ndx < servos.length; ndx++) { servos[ndx].setPosition(servoPowers[ndx]); }

            // Show data to driver
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "%.02f, %.02f, %.02f, %.02f",
                    motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            telemetry.addData("Servos", "%.02f, %.02f, %.02f, %.02f, %.02f, %.02f",
                    servoPowers[0], servoPowers[1], servoPowers[2], servoPowers[3], servoPowers[4], servoPowers[5]);
            telemetry.addData("Active Servos", "L: %d, R: %d", sL, sR);
            telemetry.update();
        }
    }

    // NOTE: 'gamepad.dpad_*' returns 'true' if if the button is actively pressed.
    // TODO: Add ABXY buttons and their actions
    private void updateBtnActions() {
        // Read current button states
        currentDPadState = new boolean[]{ gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left };

        // Reset action states
        dpadUpBtnPressed = false;
        dpadRightBtnPressed = false;
        dpadDownBtnPressed = false;
        dpadLeftBtnPressed = false;

        dpadUpBtnReleased = false;
        dpadRightBtnReleased = false;
        dpadDownBtnReleased = false;
        dpadLeftBtnReleased = false;

        // Update action states
        if(lastDPadState != null) {
            if(!lastDPadState[0] && currentDPadState[0]) { dpadUpBtnPressed = true; }
            if(!lastDPadState[1] && currentDPadState[1]) { dpadRightBtnPressed = true; }
            if(!lastDPadState[2] && currentDPadState[2]) { dpadDownBtnPressed = true; }
            if(!lastDPadState[3] && currentDPadState[3]) { dpadLeftBtnPressed = true; }

            if(lastDPadState[0] && !currentDPadState[0]) { dpadUpBtnReleased = true; }
            if(lastDPadState[1] && !currentDPadState[1]) { dpadRightBtnReleased = true; }
            if(lastDPadState[2] && !currentDPadState[2]) { dpadDownBtnReleased = true; }
            if(lastDPadState[3] && !currentDPadState[3]) { dpadLeftBtnReleased = true; }
        }

        // Save last button states
        lastDPadState = currentDPadState;
    }
}
