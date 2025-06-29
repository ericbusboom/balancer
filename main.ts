

function makeScaler (inputMin: number, inputMax: number, outputMin: number, outputMax: number) {
    // This inner function will have access to inputMin, inputMax, outputMin, outputMax
    return function (input: number): number {

        if (input < inputMin) {
            return outputMin
        }

        if (input > inputMax) {
            return outputMax
        }

        return ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
    }
}
function radToDeg (radians: number) {
    return radians * (180 / Math.PI)
}
function degToRad (degrees: number) {
    return degrees * (Math.PI / 180)
}

// Creates a PID controller function.
// 
// @param {number} kp - Proportional gain
// @param {number} ki - Integral gain
// @param {number} kd - Derivative gain
// @returns {(input: number) => number} - The PID function that takes the current error and returns the control output

function makePID (kp: number, ki: number, kd: number, alpha: number) {
    lastTime = control.millis()
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
    }
}
let lastTime = 0
let mass_angle = 0
let run = false
let count = 0
let meanAccel = 0
// Initialize the array to hold the values
let a: number[] = []
let lastError = 0
let integral = 0
let servo = servos.P1
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
// This is a 360degree servo

let kp = 3
let kd = 0.04
let ki = 0
let alpha = 0.3

let pid = makePID(kp, ki, kd, alpha)

run = false

wuKong.setLightMode(wuKong.LightMode.BREATH)


input.onButtonPressed(Button.A, function () {
    run = !(run)
    if(run){
        basic.showIcon(IconNames.Happy)
        wuKong.lightIntensity(100)
    } else {
        basic.showIcon(IconNames.No)
        wuKong.lightIntensity(0)
        
    }
})



while (true) {
    meanAccel = runningAvg(input.acceleration(Dimension.Z))
    mass_angle = radToDeg(Math.asin(meanAccel / 1024))
    
    serial.writeValue("a", mass_angle)
    

    let o = pid(mass_angle)
    serial.writeValue("o", o)

    if(run){
        wuKong.setAllMotor(o,o)
    } else {
        wuKong.setAllMotor(0, 0)
    }


    if (count-- == 0) {
        count = 1000
        basic.pause(1)
    }
}
