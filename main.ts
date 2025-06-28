// Initialize the array to hold the values
let a: number[] = [];

let servo = servos.P1


// Function to add a new value and calculate the running average
function runningAvg(x: number, l: number = 4) {
    // Add the new value to the end
    a.push(x);

    if (a.length > l) {
        a.shift();
    }

    // Calculate the sum
    let sum = 0;
    for (let i = 0; i < a.length; i++) {
        sum += a[i];
    }

    // Calculate the average
    let average = sum / a.length;

    // Return the average
    return average;
}

function radToDeg(radians: number): number {
    return radians * (180 / Math.PI);
}
function degToRad(degrees: number): number {
    return degrees * (Math.PI / 180);
}


input.onButtonPressed(Button.A, function () {
    run = !run
})

function makeScaler(
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number
): (input: number) => number {
    // This inner function will have access to inputMin, inputMax, outputMin, outputMax
    return function (input: number): number {

        if (input < inputMin) {
            return outputMin
        }

        if (input > inputMax) {
            return outputMax
        }

        return ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
    };
}

/**
 * Creates a PID controller function.
 * 
 * @param {number} kp - Proportional gain
 * @param {number} ki - Integral gain
 * @param {number} kd - Derivative gain
 * @returns {(input: number) => number} - The PID function that takes the current error and returns the control output
 */

function makePID(kp: number, ki: number, kd: number, alpha: number) {
    let integral = 0;
    let lastError = 0;
    let lastTime = control.millis();

    return function (error: number) {
        const now = control.millis();
        const dtMs = now - lastTime;
        const dt = dtMs / 1000; // Convert ms to seconds

        if(Number.isNaN(error)){
            error = lastError
        }


        // Defensive: avoid division by zero
        const safeDt = dt > 0 ? dt : 1e-3;

        integral += error * safeDt;

        let filteredError = alpha * error + (1 - alpha) * lastError;

        if (Number.isNaN(filteredError)){
            filteredError = error

        }

        const derivative = (filteredError - lastError) / dt;

        const output = kp * error + ki * integral + kd * derivative;

        lastError = error;
        lastTime = now;

        return output;
    };
}

// This is a 360degree servo
const home = 180

const servo_range = 75
const mass_range = 60


const kp = .5;
const kd = 0.04
const ki = 0.00

const alpha = .3

const pid = makePID(kp, ki, kd, alpha);

// Using the range (-1000 to 1000 ) for both input and output
// for to make it easy to interpret

// Scale from the natural IMU output ( -1024 to 1024 ) to 
// more limited range
//const imuScaler = makeScaler(-550, 550, -1000, 1000);

// Scale the output range to degrees for the servo
// remember it is a 360 deg servo, but the control is for 180
//const servoScaler = makeScaler(-mass_range, mass_range, (home - servo_range) / 2, (home + servo_range) / 2);

// This configuration just provides limits
const servoScaler = makeScaler((home - servo_range) / 2, (home + servo_range) / 2, (home - servo_range) / 2, (home + servo_range) / 2);

let meanAccel = 0;
let count = 0;

let run = true;

let servo_angle = home

while (true) {

    meanAccel = runningAvg(input.acceleration(Dimension.X))
    let mass_angle = radToDeg(Math.asin(meanAccel / 1024))

    serial.writeValue("a", mass_angle)

    let o = pid(mass_angle)
    serial.writeValue("o", o)

    servo_angle += o

    let s = servoScaler(servo_angle)
    if (!Number.isNaN(s)){
        serial.writeValue("s", s)
        if (run) {
            servo.setAngle(s) // div 2 handled by scaler
        } else {
            servo.setAngle(home / 2) // b/c 360 deg

        }
    }

   if(count-- == 0){
       count = 1000;
       basic.pause(1)
   }

}
