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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Code", group="Linear Opmode")
public class TeleOp_Code extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorRF = null;
    private DcMotor motorRB = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLF = hardwareMap.get(DcMotor.class, "motor_lf");
        motorLB = hardwareMap.get(DcMotor.class, "motor_lb");
        motorRF = hardwareMap.get(DcMotor.class, "motor_rf");
        motorRB = hardwareMap.get(DcMotor.class, "motor_rb");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double power_LF_RB, power_LB_RF;
            double power_LF, power_LB, power_RF, power_RB;

            // Calculate power based on joystick positions.
            double c_move_LR = gamepad1.right_stick_x;
            double c_move_FB = -gamepad1.right_stick_y;
            double c_rotate = gamepad1.left_stick_x;

            double c_move_mag = Math.sqrt(c_move_LR * c_move_LR + c_move_FB * c_move_FB);
            double c_move_ang = 0.0;
            if (c_move_LR == 0.0) {
                if (c_move_FB > 0.0) c_move_ang = Math.PI / 2.0;
                else if (c_move_FB < 0.0) c_move_ang = -Math.PI / 2.0;
            } else if (c_move_LR > 0.0) c_move_ang = Math.atan(c_move_FB / c_move_LR);
            else {
                c_move_ang = Math.atan(c_move_FB / c_move_LR);
                if (c_move_FB >= 0.0) c_move_ang += Math.PI;
                else c_move_ang -= Math.PI;
            }

            power_LF_RB = Math.sin(c_move_ang + Math.PI / 4.0) * c_move_mag;
            power_LB_RF = Math.sin(c_move_ang - Math.PI / 4.0) * c_move_mag;

            power_LF = Range.clip(power_LF_RB + c_rotate, -1.0, 1.0);
            power_LB = Range.clip(power_LB_RF + c_rotate, -1.0, 1.0);
            power_RF = Range.clip(power_LB_RF - c_rotate, -1.0, 1.0);
            power_RB = Range.clip(power_LF_RB - c_rotate, -1.0, 1.0);

            // Send calculated power to wheels
            motorLF.setPower(power_LF);
            motorLB.setPower(power_LB);
            motorRF.setPower(power_RF);
            motorRB.setPower(power_RB);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "L(%.2f, %.2f) R(%.2f, %.2f)", power_LF, power_LB, power_RF, power_RB);
            telemetry.update();
        }
    }
}
